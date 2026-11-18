#include <cmath>
#include <cstdio>

#include "estimation/kalman_filter.hpp"

namespace {

int failures = 0;

void expect(bool condition, const char* what, double value) {
  if (!condition) {
    std::printf("失败: %s (%.9f)\n", what, value);
    ++failures;
  }
}

constexpr double kPi = 3.14159265358979323846;

double wrap(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

// 假车的真值几何：两对装甲板不同圆不同高。
// 偶数对（id 0/2）是基准，奇数对（id 1/3）各自差 dr、dz。
struct TargetGeometry {
  double r_even{0.26};
  double dr{0.015};
  double dz{0.06};
};

// 相机在原点看向 +z，车心固定在 center。
// 板法向朝相机，所以“最正对相机”的那块板是 theta 最接近 π 的那块（cos 最负）。
int visibleId(double chassis_yaw) {
  int best = 0;
  double best_facing = 2.0;
  for (int id = 0; id < 4; ++id) {
    const double facing = std::cos(chassis_yaw + id * kPi * 0.5);
    if (facing < best_facing) {
      best_facing = facing;
      best = id;
    }
  }
  return best;
}

// 按真值几何造一份观测，和 frame_processor 里喂给 EKF 的格式一致。
// forced_id >= 0 时强制用这块板，用来构造“只露同一块板”的场景。
void makeObservation(double chassis_yaw, const TargetGeometry& geom, const Eigen::Vector3d& center, int forced_id,
                     kalfilter::armerState& armor, kalfilter::cam_ypd& ypd) {
  const int id = forced_id >= 0 ? forced_id : visibleId(chassis_yaw);
  const double theta = chassis_yaw + id * kPi * 0.5;
  const bool paired = (id % 2) == 1;
  const double r_eff = geom.r_even + (paired ? geom.dr : 0.0);

  const double x = center.x() + r_eff * std::sin(theta);
  const double y = center.y() + (paired ? geom.dz : 0.0);
  const double z = center.z() + r_eff * std::cos(theta);

  armor.measurement << x, y, z, wrap(theta);

  ypd.azi = std::atan2(x, z);
  ypd.ele = std::atan2(y, std::hypot(x, z));
  ypd.dist = std::sqrt(x * x + y * y + z * z);
}

void reportState(const kalfilter::KFResult& result) {
  std::printf("state: pos=(%.4f,%.4f,%.4f) yaw=%.4f r=%.4f dr=%.4f dz=%.4f\n", result.state(0), result.state(1),
              result.state(2), result.state(3), result.state(8), result.state(9), result.state(10));
}

/*
 * 场景 A：车体匀速旋转，两对板轮流露脸。
 * 期望：Δr/Δz 被学到真值附近，r 收敛到偶数对半径，车心位置不被结构差异带偏。
 */
void runRotatingPair() {
  const TargetGeometry geom;
  const Eigen::Vector3d center(0.0, 0.0, 3.0);

  kalfilter::KalmanConfig config;
  // 这是无噪声的合成场景，R 用理想值就行。工程默认的 R 是按真实数据定的，
  // 直接拿来会把增益压得很小，Δr 收敛慢到几百帧都看不出结论。
  config.ele = 2e-3;
  config.azi = 2e-3;
  config.k_dist = 0.005;
  config.yaw = 5e-3;
  kalfilter::KalmanFilter filter(config);

  constexpr double dt = 0.02;
  constexpr double omega = 2.5;

  // 起始 yaw 取 π，让偶数 id 0 的板正对相机，init 的锚点落在基准对上。
  double chassis_yaw = kPi;

  kalfilter::armerState armor;
  kalfilter::cam_ypd ypd;
  makeObservation(chassis_yaw, geom, center, -1, armor, ypd);
  filter.init(armor);

  constexpr int frames = 300;
  for (int i = 0; i < frames; ++i) {
    chassis_yaw = wrap(chassis_yaw + omega * dt);
    filter.predict(dt);
    makeObservation(chassis_yaw, geom, center, -1, armor, ypd);
    filter.update(armor, ypd, Eigen::Matrix3d::Identity());
    filter.renewArmorList();
  }

  const auto result = filter.getOutput();
  expect(result.valid, "旋转跟踪后应有效", 0.0);
  if (!result.valid) {
    reportState(result);
    return;
  }

  const Eigen::Vector3d pos(result.state(0), result.state(1), result.state(2));
  const double pos_err = (pos - center).norm();
  const double r_err = std::abs(result.state(8) - geom.r_even);
  const double dr_err = std::abs(result.state(9) - geom.dr);
  const double dz_err = std::abs(result.state(10) - geom.dz);

  expect(std::isfinite(result.state.sum()), "状态应全部有限", result.state.sum());
  expect(pos_err < 0.015, "车心位置应贴近真值", pos_err);
  expect(r_err < 0.008, "r 应收敛到偶数对半径", r_err);
  expect(dr_err < 0.003, "Δr 应收敛到真值", dr_err);
  expect(dz_err < 0.008, "Δz 应收敛到真值", dz_err);

  if (failures > 0) reportState(result);
}

/*
 * 场景 B：只露同一块（偶数对）板。
 * Δr/Δz 在观测里根本没有梯度，只该停在前验附近，不能自己漂出去。
 */
void runSinglePair() {
  const TargetGeometry geom;
  const Eigen::Vector3d center(0.0, 0.0, 3.0);

  kalfilter::KalmanConfig config;
  kalfilter::KalmanFilter filter(config);

  constexpr double dt = 0.02;
  constexpr double chassis_yaw = kPi;  // 静止

  kalfilter::armerState armor;
  kalfilter::cam_ypd ypd;
  makeObservation(chassis_yaw, geom, center, 0, armor, ypd);
  filter.init(armor);

  constexpr int frames = 100;
  for (int i = 0; i < frames; ++i) {
    filter.predict(dt);
    makeObservation(chassis_yaw, geom, center, 0, armor, ypd);
    filter.update(armor, ypd, Eigen::Matrix3d::Identity());
    filter.renewArmorList();
  }

  const auto result = filter.getOutput();
  expect(result.valid, "单板跟踪后应有效", 0.0);
  if (!result.valid) {
    reportState(result);
    return;
  }

  expect(std::abs(result.state(9)) < 0.005, "只露一对板时 Δr 不该乱跑", result.state(9));
  expect(std::abs(result.state(10) - config.init_dz) < 0.01, "只露一对板时 Δz 不该乱跑", result.state(10));

  if (failures > 0) reportState(result);
}

}  // namespace

int main() {
  runRotatingPair();
  runSinglePair();

  if (failures == 0) {
    std::printf("kalman_armor_pair_test 通过\n");
    return 0;
  }
  return 1;
}
