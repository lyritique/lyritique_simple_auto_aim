#include "sensors/device_clock.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <random>

namespace {

// 这台相机的名义帧周期：60 fps 下每帧 1,667,091 tick（实测值）。
constexpr std::uint64_t kTickPerFrame = 1667091;

// 相机实际只跑到 56 fps 时的真实帧周期。
constexpr double kFramePeriodTrue = 17.86e-3;

// 真实斜率：秒每 tick，≈10.71 ns。按 10 ns 用就会以 7% 的速率往前漂。
constexpr double kScaleTrue = kFramePeriodTrue / static_cast<double>(kTickPerFrame);

// 先验：代码里的 tick_ns 默认值。
constexpr double kScalePrior = 1.0e-8;

}  // namespace

int main() {
  camera::DeviceClock clock;

  // 相机的计数器是开机横走的，模拟跑了一个多小时之后的绝对值。
  const std::uint64_t ticks0 = 6'000'000'000ULL;
  const double host0 = 1000.0;

  // 冷启动时拿先验斜率算出的截距：在 ticks0 这一点是对的，之后会随漂移变坏。
  clock.reset(kScalePrior, host0 - kScalePrior * static_cast<double>(ticks0));

  std::mt19937 rng(20261119);
  std::uniform_real_distribution<double> jitter(-0.002, 0.002);

  std::uint64_t ticks = ticks0;
  double host = host0;
  double prev_stamp = 0.0;
  double min_step = 1e9;
  double max_step = 0.0;
  double scale_at_300 = 0.0;

  for (int k = 0; k < 600; ++k) {
    clock.push(ticks, host);
    // 20 ms 滞后，低于冻结阈值；模拟的是「不积压、只是收帧晚一点」的正常情况。
    const auto report = clock.update(ticks, 0.02);
    if (report.model_broken) {
      std::cerr << "线性模型被误判失效，第 " << k << " 帧残差=" << report.residual_ms << " ms\n";
      return 1;
    }

    const double stamp = clock.scale() * static_cast<double>(ticks) + clock.offset();
    if (k > 0) {
      const double step = stamp - prev_stamp;
      min_step = std::min(min_step, step);
      max_step = std::max(max_step, step);
    }
    prev_stamp = stamp;

    if (k == 300) scale_at_300 = clock.scale();

    ticks += kTickPerFrame;
    // 曝光时刻是晶振锁的，只有传输延迟在抖，所以 host 必须由帧号算出来，
    // 不能在上一次的基础上累加——累加会变成随机游走，那是错的模型。
    host = host0 + static_cast<double>(k + 1) * kFramePeriodTrue + jitter(rng);
    // 中途卡一下：画图线程抢 CPU 时真实会掉 200 ms。离群点必须被 MAD 剔掉。
    if (k == 300) host += 0.200;
  }

  // 第一关：斜率要收敛到真实值。允许 0.5%：窗口 5 s、抖动 2 ms，理论误差远小于这个数。
  if (std::abs(clock.scale() / kScaleTrue - 1.0) > 5e-3) {
    std::cerr << "斜率没收敛: fit=" << clock.scale() * 1e9 << " ns/tick, true=" << kScaleTrue * 1e9
              << " ns/tick\n";
    return 2;
  }
  if (std::abs(scale_at_300 / kScaleTrue - 1.0) > 5e-3) {
    std::cerr << "第 300 帧还没收敛: fit=" << scale_at_300 * 1e9 << " ns/tick\n";
    return 3;
  }

  // 第二关：换斜率那一帧 stamp 不能跳。正常一帧 17.86 ms，
  // 平滑和重锚一起工作的话，单帧步长应该始终落在这个量级附近。
  if (min_step < 0.010 || max_step > 0.030) {
    std::cerr << "stamp 不连续: min_step=" << min_step * 1e3 << " ms max_step=" << max_step * 1e3 << " ms\n";
    return 4;
  }

  // 第三关：截距要收敛到真实曝光时刻，而不是停在冷启动时那个值。
  // 真实 b = host0 + 漂移 - scale*ticks，这里直接比「映射时刻和收帧时刻的差不涨」。
  const double lag_end = host - (clock.scale() * static_cast<double>(ticks) + clock.offset());
  if (std::abs(lag_end) > 0.05) {
    std::cerr << "截距没收敛: 末端 lag=" << lag_end * 1e3 << " ms\n";
    return 5;
  }

  // 第四关：样本不够时不许出结果。
  camera::DeviceClock cold;
  cold.reset(kScalePrior, 0.0);
  for (int k = 0; k < 20; ++k) cold.push(static_cast<std::uint64_t>(k) * kTickPerFrame, k * kFramePeriodTrue);
  if (cold.update(20 * kTickPerFrame, 0.02).updated) {
    std::cerr << "样本不足也给了结果\n";
    return 6;
  }

  // 第五关：正在积压时必须冻结，否则会把排队时间当成速率吸收进斜率。
  camera::DeviceClock busy;
  busy.reset(kScalePrior, 0.0);
  for (int k = 0; k < 100; ++k) busy.push(static_cast<std::uint64_t>(k) * kTickPerFrame, k * kFramePeriodTrue);
  if (busy.update(100 * kTickPerFrame, 0.8).updated) {
    std::cerr << "积压时仍然更新了斜率\n";
    return 7;
  }

  // 第六关：斜率和截距都改了之后，映射结果必须仍然单调递增。
  double last = 0.0;
  for (int k = 0; k < 10; ++k) {
    const double t = clock.scale() * static_cast<double>(ticks + k * kTickPerFrame) + clock.offset();
    if (k > 0 && t <= last) {
      std::cerr << "映射结果不单调\n";
      return 8;
    }
    last = t;
  }

  std::cout << "device clock test passed: fit=" << clock.scale() * 1e9 << " ns/tick (true "
            << kScaleTrue * 1e9 << "), step=[" << min_step * 1e3 << ", " << max_step * 1e3 << "] ms\n";
  return 0;
}
