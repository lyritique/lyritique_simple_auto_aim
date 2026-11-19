#ifndef AUTOAIM_SENSORS_DEVICE_CLOCK_HPP
#define AUTOAIM_SENSORS_DEVICE_CLOCK_HPP

#include <cstddef>
#include <cstdint>
#include <deque>

namespace camera {

/*
 * 设备戳 -> 秒 的在线标定。
 *
 * 这台 U3V 相机的 nDevTimeStamp 不是自由计数器，而是「帧号 × 1/设定帧率」算出来的
 * 名义时间：相邻帧的 tick 差恒为 1,667,091（±1 取整），相机实际只跑到 56 fps 时也不变。
 * 于是按 10 ns/tick 映射出来的时刻会以 (设定帧率/实际帧率 - 1) 的速率往前漂，
 * 一路漂到秒级，最后撞上 mvs_camera 的 1 秒回退分支，延迟指标全部失真。
 *
 * 这里用「设备戳 vs 主机收帧时刻」的线性回归把真实斜率量出来：
 *     host_k = scale * ticks_k + offset + 噪声
 * 窗口滑动、斜率低通；换斜率时重锚 offset，保证映射出来的时刻连续。
 */
class DeviceClock {
 public:
  struct Options {
    // 窗口上限，≈5 s @60 fps。太短斜率被收帧抖动带偏，太长跟不上相机帧率变化。
    std::size_t capacity{300};
    // 少于此数不出结果，≈0.75 s。这就是冷启动那个固定窗口，只是不冻结、继续长。
    std::size_t min_samples{45};
    // 窗口内设备时间跨度下限，太短的话分母小、斜率没意义。
    double min_span_s{0.5};
    // 斜率低通系数，每帧一次，τ≈1/smooth 帧。
    double smooth{0.02};
    // 单次拟合相对跳变超过它就本次作废，避免一个坏窗口把斜率拽走。
    double max_scale_jump{0.03};
    // 截距每次最多挪多少秒。截距一动 stamp 就跳，必须限速。
    double max_offset_step_s{5e-4};
    // 剔除离群后的残差 RMS 上限。超过说明线性模型本身不成立（比如真积压了）。
    double max_residual_ms{10.0};
    // 连续几次超限就判定模型失效，调用方应退回主机时间。
    int max_bad_fits{5};
    // 当前滞后超过它就不拟合：窗口里混进了积压，拟合会把延迟当成速率吸收掉。
    // 不能给太小 —— 斜率从先验收敛到真值要一两秒，这期间滞后会自己涨到 90 ms 左右
    // （60 fps 下 7% 漂移、τ≈50 帧，总计约 60 ms），阈值卡在 100 ms 会把自己冻死。
    double freeze_lag_s{0.5};
    // 斜率物理范围，约等于 9~12 ns/tick。
    double scale_lo{9.0e-9};
    double scale_hi{1.2e-8};
  };

  struct Report {
    bool updated{};       // 本次是否真的重估并写回
    bool model_broken{};  // 连续超限，调用方应改用主机时间
    std::size_t samples{};
    double span_s{};      // 窗口内设备时间跨度
    double scale{};       // 当前（平滑后）使用的斜率
    double fit_scale{};   // 本窗口的原始拟合值，用来对比和打印
    double residual_ms{}; // 剔除离群后的残差 RMS
  };

  DeviceClock() = default;
  explicit DeviceClock(const Options& options) : options_(options) {}

  // 用冷启动标定的结果当先验。约定 stamp = scale * ticks + offset。
  void reset(double scale, double offset);

  // 每帧喂一个样本，ticks 必须递增。
  void push(std::uint64_t ticks, double host);

  // 用当前窗口重估一次。lag_s 是当前帧的「收帧时刻 - 映射时刻」，用来判断有没有积压。
  Report update(std::uint64_t now_ticks, double lag_s);

  double scale() const { return scale_; }
  double offset() const { return offset_; }
  std::size_t samples() const { return samples_.size(); }

 private:
  struct Sample {
    std::uint64_t ticks{};
    double host{};
  };

  // 最小二乘。x 先用均值中心化，否则 1e10 量级的 tick 会把有效位数吃掉。
  static bool fitLine(const Sample* pts, std::size_t count, double& slope, double& intercept);

  Options options_{};
  std::deque<Sample> samples_;
  double scale_{1e-8};
  double offset_{};
  // 上一次被接受的拟合值。跳变护栏拿它比，不能拿平滑后的 scale_ 比：
  // 平滑本身每帧只走 alpha，拿 scale_ 当基准的话第一次更新之后就被自己锁死了。
  double target_scale_{1e-8};
  // 拟合成功过一次之后才启用跳变护栏：先验 10 ns/tick 可能和真实值差 7%，
  // 第一次必须放进来，否则永远收敛不了。
  bool have_fit_{};
  int bad_fits_{};
  bool model_broken_{};
};

}  // namespace camera

#endif
