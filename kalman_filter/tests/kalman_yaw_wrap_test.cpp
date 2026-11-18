#include <cmath>
#include <cstdio>

#include "estimation/kalman_filter.hpp"

// 作用：确认自旋时 yaw 不会跑出 ±π。实现：给一个大角速度连续预测，检查状态是否被折回。
int main() {
  kalfilter::KalmanConfig config;
  kalfilter::KalmanFilter filter(config);

  kalfilter::armerState measurement;
  // 朝向接近 π，再以 3 rad/s 自旋，几帧就会越界。
  measurement.measurement << 0.0, 0.0, 2.0, 3.0;
  filter.init(measurement);

  for (int i = 0; i < 40; ++i) {
    filter.predict(0.05);
    const auto state = filter.getOutput().state;
    if (!std::isfinite(state(3)) || std::abs(state(3)) > M_PI + 1e-9) {
      std::printf("失败: 第 %d 帧 yaw=%f 超出 ±π\n", i, state(3));
      return 1;
    }
  }

  std::printf("kalman_yaw_wrap_test 通过\n");
  return 0;
}
