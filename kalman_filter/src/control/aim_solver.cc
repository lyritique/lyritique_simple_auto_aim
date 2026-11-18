#include "control/aim_solver.hpp"

#include <algorithm>
#include <cmath>

namespace control {
namespace {

constexpr double kPi = 3.14159265358979323846;

// 相机系（x 右、y 下、z 前）-> C 板 IMU 体轴（x 前、y 左、z 上）。
// C 板四元数描述的是体轴：它报的 yaw/pitch 就是体轴 +x 的方位角/仰角（实测两帧精确到 0.01°）。
// EKF 的观测向量在相机系里，直接乘四元数会差一个 90°，所以要先换轴。
const Eigen::Matrix3d kCameraToBody =
    (Eigen::Matrix3d() << 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0).finished();

double wrap(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double clampAbs(double v, double limit) { return std::clamp(v, -limit, limit); }

// 第 id 块板的水平方位角。EKF 里 id 0 的中心 yaw 就是 X(3)，每块差 90°。
double armorTheta(double yaw, int id) { return yaw + id * kPi * 0.5; }

// 挑“正对相机”的那块板：相机在 C0 原点，从目标看过去方位是 atan2(-x, -z)；
// 取方位差最小的那块，等价于法向最朝向相机的板。打旋转目标时要的是命中时刻
// 露出来的那块，所以用预测后的 yaw 来选。
int pickArmor(const Eigen::Vector3d& center, double yaw) {
  const double to_camera = std::atan2(-center.x(), -center.z());
  int best = 0;
  double best_gap = 1e9;
  for (int id = 0; id < 4; ++id) {
    const double gap = std::abs(wrap(armorTheta(yaw, id) - to_camera));
    if (gap < best_gap) {
      best_gap = gap;
      best = id;
    }
  }
  return best;
}

// 板中心：半径按奇偶 id 叠加 Δr，竖直方向叠加 Δz，和观测模型完全一致。
Eigen::Vector3d armorCenter(const kalfilter::KFResult& state, const Eigen::Vector3d& center, double yaw, int id) {
  const bool paired = (id % 2) == 1;
  const double radius = state.state(8) + (paired ? state.state(9) : 0.0);
  const double theta = armorTheta(yaw, id);

  Eigen::Vector3d point;
  point.x() = center.x() + radius * std::sin(theta);
  point.y() = center.y() + (paired ? state.state(10) : 0.0);
  point.z() = center.z() + radius * std::cos(theta);
  return point;
}

}  // namespace

AimSolver::AimSolver(const AimConfig& config) : config_(config) {}

void AimSolver::reset() {
  has_last_ = false;
  last_time_ = 0.0;
  last_yaw_ = 0.0;
  last_pitch_ = 0.0;
  last_yaw_vel_ = 0.0;
  last_pitch_vel_ = 0.0;
}

AimSolution AimSolver::solve(const kalfilter::KFResult& state, const Eigen::Quaterniond& q_init,
                             const Eigen::Quaterniond& q_now, double latency_seconds, double bullet_speed,
                             double now_seconds, const Eigen::Matrix3d* c0_to_world) {
  AimSolution solution;
  if (!state.valid) return solution;

  const double speed = bullet_speed > 1.0 ? bullet_speed : config_.bullet_speed;
  const Eigen::Vector3d position = state.state.head<3>();
  const Eigen::Vector3d velocity = state.state.segment<3>(4);

  // 飞行时间 t 和提前量互相耦合（距离里也含着提前量），迭代两次足够收敛。
  double flight_time = 0.0;
  Eigen::Vector3d point = position;
  double yaw = state.state(3);
  int armor_id = 0;

  for (int iteration = 0; iteration < 2; ++iteration) {
    const double t = latency_seconds + flight_time;
    const Eigen::Vector3d center = position + velocity * t;
    yaw = wrap(state.state(3) + state.state(7) * t);

    armor_id = pickArmor(center, yaw);
    point = armorCenter(state, center, yaw, armor_id);

    flight_time = point.norm() / speed;
  }

  // C0 -> C 板世界系。
  // 默认：C0 = 首帧相机系，q_init 把它转回首帧的姿态，q_now^-1 再退到当前相机系，
  //       换轴成体轴后用 q_now 转到世界系。三条连起来就是：q_now · M · p_cam。
  // level：C0 已经是水平世界，直接乘 E→C 板世界系，不再经过相机姿态。
  Eigen::Vector3d point_world;
  if (c0_to_world != nullptr) {
    point_world = (*c0_to_world) * point;
  } else if (config_.cam_remap) {
    const Eigen::Vector3d point_camera = q_now.conjugate() * (q_init * point);
    point_world = q_now * (kCameraToBody * point_camera);
  } else {
    point_world = q_init * point;
  }

  // c0_to_world 指向的 C 板世界系固定是 z 朝天；此时不要再按 --up-axis 换轴。
  const bool z_up = (c0_to_world != nullptr) || config_.z_up;

  if (config_.compensate_drop) {
    // 沿世界系的竖轴抬高一点补偿下坠；未标定前别开。
    const double drop = 0.5 * config_.gravity * flight_time * flight_time;
    if (z_up) {
      point_world.z() += drop;
    } else {
      point_world.y() += drop;
    }
  }

  const Eigen::Vector3d dir = point_world.normalized();
  solution.valid = true;
  solution.point_c0 = point;
  solution.dir_world = dir;
  // 世界系约定：y 朝天时水平面是 x-z，z 朝天时是 x-y。yaw 一律取"水平面内绕竖轴的方位角"。
  if (z_up) {
    solution.yaw = config_.yaw_sign * std::atan2(dir.y(), dir.x()) + config_.yaw_offset;
    solution.pitch = config_.pitch_sign * std::asin(std::clamp(dir.z(), -1.0, 1.0)) + config_.pitch_offset;
  } else {
    solution.yaw = config_.yaw_sign * std::atan2(dir.x(), dir.z()) + config_.yaw_offset;
    solution.pitch = config_.pitch_sign * std::asin(std::clamp(dir.y(), -1.0, 1.0)) + config_.pitch_offset;
  }
  solution.yaw = wrap(solution.yaw);
  solution.distance = point.norm();
  solution.flight_time = flight_time;
  solution.armor_id = armor_id;

  // 角速度/角加速度靠相邻两帧指令差分，给 C 板当前馈。
  // 差分本身放大了抖动，所以做一阶低通，并且限幅到物理上说得过去的范围。
  if (has_last_) {
    const double dt = now_seconds - last_time_;
    if (dt > 1e-4 && dt < 0.5) {
      const double yaw_rate_raw = wrap(solution.yaw - last_yaw_) / dt;
      const double pitch_rate_raw = (solution.pitch - last_pitch_) / dt;

      constexpr double kAlpha = 0.5;
      const double yaw_rate = kAlpha * yaw_rate_raw + (1.0 - kAlpha) * last_yaw_vel_;
      const double pitch_rate = kAlpha * pitch_rate_raw + (1.0 - kAlpha) * last_pitch_vel_;

      solution.yaw_vel = clampAbs(yaw_rate, 10.0);
      solution.pitch_vel = clampAbs(pitch_rate, 10.0);
      solution.yaw_acc = clampAbs((solution.yaw_vel - last_yaw_vel_) / dt, 100.0);
      solution.pitch_acc = clampAbs((solution.pitch_vel - last_pitch_vel_) / dt, 100.0);
    }
  }

  has_last_ = true;
  last_time_ = now_seconds;
  last_yaw_ = solution.yaw;
  last_pitch_ = solution.pitch;
  last_yaw_vel_ = solution.yaw_vel;
  last_pitch_vel_ = solution.pitch_vel;

  return solution;
}

}  // namespace control
