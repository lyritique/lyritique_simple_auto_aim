#include "timing/clock_sync.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace kalfilter {
namespace {

// 作用：微秒转秒的口粮。实现：就是 1e-6，免得代码里到处撒魔法数字。
constexpr double kMicrosecond = 1e-6;
// 作用：32 位无符号计数的总格数，回绕一次补这么多。实现：1ULL 左移 32 位，注意用 ULL 防止 32 位平台上溢出。
constexpr std::uint64_t kUint32Range = 1ULL << 32;
// 作用：半个圈，用来判断“变小”到底是回绕还是后退。实现：1ULL 左移 31 位，差值过半圈就认定是绕圈。
constexpr std::uint64_t kHalfUint32Range = 1ULL << 31;

}  // namespace

// 作用：把参数收拾干净后存起来。实现：成员初始化列表里各夹一次合法范围，延迟不小于 0，窗口至少 1。
ClockSync::ClockSync(double comm_latency_seconds, std::size_t max_observations)
    : comm_latency_seconds_(std::max(0.0, comm_latency_seconds)),
      max_observations_(std::max<std::size_t>(1, max_observations)) {}

// 作用：热更新通信延迟。实现：先拿锁再赋值，同时把负值夹成 0，防止有人填了个“提前到达”。
void ClockSync::setCommLatency(double seconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  comm_latency_seconds_ = std::max(0.0, seconds);
}

// 作用：读通信延迟。实现：锁内拷贝一份再返回，锁外随便看。
double ClockSync::commLatency() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return comm_latency_seconds_;
}

// 作用：喂一个板卡时间戳，换回展开后的微秒数和对应的上位机时间。实现：先做输入体检，再展开回绕、验单调、更新
// offset，四步一条流水线。
bool ClockSync::observeAndMap(float board_stamp_us, double receive_host_time, uint64_t& unwrapped_board_us,
                              double& event_host_time) {
  // 作用：挡住 NaN、负数时间和无穷大，这些进了计算只会污染 offset。实现：三个 isfinite/大小判断，不合规直接退票。
  if (!std::isfinite(board_stamp_us) || board_stamp_us < 0.0F || !std::isfinite(receive_host_time)) {
    return false;
  }

  // 作用：把浮点戳四舍五入成整数微秒，顺便防止 float 精度带来的半微秒漂移。实现：llround 到 long long。
  const auto rounded = std::llround(board_stamp_us);
  // 作用：算一个“上界 + 1”，允许恰好等于 2^32 的边界值。实现：把 2^32 转成有符号数，方便和 rounded 比大小。
  const auto max_plus_one = static_cast<long long>(kUint32Range);
  // 作用：拦下超出 32 位计数范围的值，否则强转就是未定义行为。实现：上下界一夹，越界返回 false。
  if (rounded < 0 || rounded > max_plus_one) {
    return false;
  }

  // 作用：从这里开始动共享状态，先上锁。实现：lock_guard 管到函数结束，异常也能自动解锁。
  std::lock_guard<std::mutex> lock(mutex_);

  // 作用：得到真正的 32 位原始计数。实现：恰好等于 2^32 时归零（等价于回绕后的 0），其余直接强转 uint32。
  const std::uint32_t raw = rounded == max_plus_one ? 0 : static_cast<std::uint32_t>(rounded);

  // 作用：第一次见面的初始化，立好基准免得拿空气做差。实现：记下首帧 raw，回绕补偿从 0 起。
  if (!has_raw_) {
    has_raw_ = true;
    last_raw_us_ = raw;
    wrap_offset_us_ = 0;
  }
  // 作用：检测 32 位计数器是否绕圈了。实现：数值变小且老值减新值超过半圈，说明是回绕不是后退，补偿加一整圈。
  else if (raw<last_raw_us_&& static_cast<std::uint64_t>(last_raw_us_ - raw)> kHalfUint32Range) {
    wrap_offset_us_ += kUint32Range;
  }

  // 作用：把补偿和历史拼成 64 位单调时间。实现：回绕补偿 + 当前 raw，时间轴从此不会倒流 71 分钟。
  const std::uint64_t candidate = wrap_offset_us_ + static_cast<std::uint64_t>(raw);

  // 乱序或重复的 C 板时间戳不能写进单调历史。
  // 作用：拒绝迟到的包，保护历史严格递增。实现：候选值不大于上一个展开值就 return false，说明是重复或乱序。
  if (has_unwrapped_ && candidate <= last_unwrapped_us_) return false;

  // 作用：确认收货，把候选值登记成新的历史末端。实现：一次写入输出参数、三个成员，原子地完成状态推进。
  unwrapped_board_us = candidate;
  last_raw_us_ = raw;
  last_unwrapped_us_ = candidate;
  has_unwrapped_ = true;

  // 作用：算出这次观测的时间偏移——事件真正发生的上位机时刻减去板卡时刻。实现：到达时间先减通信延迟，再减板卡秒数。
  const double observation =
      (receive_host_time - comm_latency_seconds_) - static_cast<double>(candidate) * kMicrosecond;
  // 作用：把这次观测投进中位数窗口，更新全局 offset。实现：交给 updateOffsetUnlocked 处理。
  updateOffsetUnlocked(observation);

  // 作用：用刚更新的 offset 把板卡时间翻译成上位机时间。实现：微秒乘 1e-6 变秒，再加 offset。
  event_host_time = static_cast<double>(candidate) * kMicrosecond + offset_seconds_;
  return true;
}

// 作用：只查表不喂数据，把历史板卡时间换算成上位机时间。实现：锁内确认 offset 已就绪，然后一次乘加。
bool ClockSync::boardToHost(uint64_t unwrapped_board_us, double& event_host_time) const {
  std::lock_guard<std::mutex> lock(mutex_);
  // 作用：没标定过就没有换算资格。实现：has_offset_ 为假直接返回 false，输出参数保持原样。
  if (!has_offset_) return false;

  // 作用：核心换算公式，板卡微秒 -> 上位机秒。实现：乘 1e-6 换单位，加 offset 补时差。
  event_host_time = static_cast<double>(unwrapped_board_us) * kMicrosecond + offset_seconds_;
  return true;
}

// 作用：对外汇报“我能不能换算”。实现：锁内读 has_offset_ 一个 bool。
bool ClockSync::hasOffset() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return has_offset_;
}

// 作用：失忆重启，用于重连、换板子或时钟漂了太多。实现：锁内把开关、计数器、回绕补偿、历史队列、offset 统统归零。
void ClockSync::reset() {
  std::lock_guard<std::mutex> lock(mutex_);
  has_raw_ = false;
  has_unwrapped_ = false;
  last_raw_us_ = 0;
  wrap_offset_us_ = 0;
  last_unwrapped_us_ = 0;
  offsets_.clear();
  offset_seconds_ = 0.0;
  has_offset_ = false;
}

// 作用：滚动维护 offset 的中位数，让单次抖动无处发作。实现：先入队裁窗，再 nth_element 求中位，最后点亮 has_offset_。
void ClockSync::updateOffsetUnlocked(double observation) {
  // 作用：新观测入队。实现：push_back 到队尾。
  offsets_.push_back(observation);
  // 作用：窗口超员时请最老的出去。实现：while 循环 pop_front，直到 size 回到上限。
  while (offsets_.size() > max_observations_) {
    offsets_.pop_front();
  }

  // 作用：拷贝一份再排序，别动原来的队列顺序。实现：用 vector 迭代器区间构造。
  std::vector<double> sorted(offsets_.begin(), offsets_.end());
  // 作用：定位中位数该在的位置。实现：begin 加 size/2，偶数个时取偏右那个，省得讨论两值平均。
  const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
  // 作用：只把中位数放到正确位置，其余不管，够用还快。实现：nth_element，平均 O(n)。
  std::nth_element(sorted.begin(), middle, sorted.end());
  // 作用：把中位数升格为当前 offset。实现：解引用 middle 直接赋值。
  offset_seconds_ = *middle;
  // 作用：宣布 offset 可用，之后 boardToHost 才肯干活。实现：置 true。
  has_offset_ = true;
}

}  // namespace kalfilter
