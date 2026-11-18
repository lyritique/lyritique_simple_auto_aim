#include <cmath>
#include <cstdio>

#include "control/aim_solver.hpp"

namespace {

int failures = 0;

void expect(bool condition, const char* what, double value) {
  if (!condition) {
    std::printf("失败: %s (%.9f)\n", what, value);
    ++failures;
  }
}

constexpr double kPi = 3.14159265358979323846;

kalfilter::KFResult makeState(double yaw, const Eigen::Vector3d& velocity) {
  kalfilter::KFResult state;
  state.valid = true;
  state.state.setZero();
  // X=(px,py,pz,yaw,vx,vy,vz,vyaw,r,Δr,Δz)，目标放在正前方 2 m。
  state.state(2) = 2.0;
  state.state(3) = yaw;
  state.state.segment<3>(4) = velocity;
  state.state(8) = 0.254;
  state.state(9) = 0.02;   // Δr
  state.state(10) = 0.03;  // Δz
  return state;
}

// 作用：相机水平时，指令就是几何方向。实现：q_init 单位旋转，目标在正前方。
void horizontalCamera() {
  control::AimSolver solver;
  const auto solution = solver.solve(makeState(0.0, Eigen::Vector3d::Zero()), Eigen::Quaterniond::Identity(),
                                     Eigen::Quaterniond::Identity(), 0.0, 25.0, 0.0);

  expect(solution.valid, "状态有效时应出解", 0.0);
  // 正对相机的是 id 2（偶数对），板心在 z = 2 - r。
  expect(solution.armor_id == 2, "应选中正对相机的第 2 块板", solution.armor_id);
  expect(std::abs(solution.yaw) < 1e-9, "正前方 yaw 应为 0", solution.yaw);
  expect(std::abs(solution.pitch) < 1e-9, "正前方 pitch 应为 0", solution.pitch);
  expect(std::abs(solution.distance - (2.0 - 0.254)) < 1e-9, "距离应为板心距离", solution.distance);
}

/*
 * 相机耷拉 20°：C 板体轴里光轴是 +x，低头就是绕体轴 +y 转 20°。
 * 目标就在光轴上，正确的世界系指令是 pitch = -20°。
 * 这一条就是"发出去的是绝对世界角、耷拉角必须被抵消"的判据。
 */
void droopedCamera() {
  const Eigen::Quaterniond droop(Eigen::AngleAxisd(20.0 * kPi / 180.0, Eigen::Vector3d::UnitY()));

  control::AimSolver solver;
  const auto solution = solver.solve(makeState(0.0, Eigen::Vector3d::Zero()), droop, droop, 0.0, 25.0, 0.0);

  const double expected = -20.0 * kPi / 180.0;
  expect(solution.valid, "耷拉相机也应出解", 0.0);
  expect(std::abs(solution.yaw) < 1e-9, "耷拉只影响 pitch，yaw 仍应为 0", solution.yaw);
  expect(std::abs(solution.pitch - expected) < 1e-6, "耷拉角必须被补偿进指令", solution.pitch - expected);
}

// 作用：pitch 正方向可翻。实现：同一个场景给 pitch_sign=-1，符号应整体取反。
void flipPitchSign() {
  control::AimConfig config;
  config.pitch_sign = -1.0;
  control::AimSolver solver(config);

  const Eigen::Quaterniond droop(Eigen::AngleAxisd(20.0 * kPi / 180.0, Eigen::Vector3d::UnitY()));
  const auto solution = solver.solve(makeState(0.0, Eigen::Vector3d::Zero()), droop, droop, 0.0, 25.0, 0.0);

  expect(std::abs(solution.pitch - 20.0 * kPi / 180.0) < 1e-6, "pitch_sign=-1 应整体取反", solution.pitch);
}

// 作用：横向速度加延迟应产生一个提前角。实现：vx=1 m/s、延迟 0.1 s，yaw 应为正。
void leadCompensation() {
  control::AimSolver solver;
  const auto solution =
      solver.solve(makeState(0.0, Eigen::Vector3d(1.0, 0.0, 0.0)), Eigen::Quaterniond::Identity(),
                    Eigen::Quaterniond::Identity(), 0.1, 25.0, 0.0);

  expect(solution.valid, "有速度时也应出解", 0.0);
  // 相机 +x 是右，换到 C 板体轴是 -y，而 yaw=atan2(y,x)，所以目标往右 yaw 变负。
  expect(solution.yaw < -0.05, "目标往右时 yaw 应为负", solution.yaw);
  // 提前量 + 飞行时间一共约 0.17 s，距离该比 1.75 m 长出十几厘米。
  expect(solution.distance > 2.0 - 0.254, "提前量应把瞄准点推到前方", solution.distance);
}

// 作用：yaw 正方向可翻。实现：同一个横向场景给 yaw_sign=-1，提前角应整体取反。
void flipYawSign() {
  control::AimConfig config;
  config.yaw_sign = -1.0;
  control::AimSolver solver(config);

  const auto solution =
      solver.solve(makeState(0.0, Eigen::Vector3d(1.0, 0.0, 0.0)), Eigen::Quaterniond::Identity(),
                    Eigen::Quaterniond::Identity(), 0.1, 25.0, 0.0);
  expect(solution.yaw > 0.05, "yaw_sign=-1 应整体取反", solution.yaw);
}

// 作用：奇数 id 的板要用 r+Δr 和 y+Δz。实现：车体转 90°，此时正对相机的是 id 1。
void pairedArmorOffset() {
  control::AimSolver solver;
  const auto solution = solver.solve(makeState(kPi * 0.5, Eigen::Vector3d::Zero()), Eigen::Quaterniond::Identity(),
                                     Eigen::Quaterniond::Identity(), 0.0, 25.0, 0.0);

  expect(solution.armor_id == 1, "转 90° 后应选中第 1 块板", solution.armor_id);
  expect(std::abs(solution.point_c0.y() - 0.03) < 1e-9, "奇数对应叠加 Δz", solution.point_c0.y());
  expect(std::abs(solution.point_c0.z() - (2.0 - 0.254 - 0.02)) < 1e-9, "奇数对应叠加 Δr",
         solution.point_c0.z());
}

// 作用：状态不可信时不能瞎发。实现：valid=false 应返回 valid=false。
void invalidState() {
  control::AimSolver solver;
  kalfilter::KFResult state;
  const auto solution = solver.solve(state, Eigen::Quaterniond::Identity(), Eigen::Quaterniond::Identity(), 0.0, 25.0, 0.0);
  expect(!solution.valid, "状态无效时解也应无效", 0.0);
}

}  // namespace

int main() {
  horizontalCamera();
  droopedCamera();
  flipPitchSign();
  leadCompensation();
  flipYawSign();
  pairedArmorOffset();
  invalidState();

  if (failures == 0) {
    std::printf("aim_solver_test 通过\n");
    return 0;
  }
  return 1;
}
