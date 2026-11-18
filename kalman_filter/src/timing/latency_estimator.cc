#include "timing/latency_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace kalfilter {

// 作用：记下窗口大小，顺手把 0 修成 1。实现：构造函数初始化列表里一次 max 兜底，从此窗口不会空。
LatencyEstimator::LatencyEstimator(std::size_t window_size) : window_size_(std::max<std::size_t>(1, window_size)) {}

// 作用：把一帧的处理延迟（曝光到命令就绪）记入统计。实现：终点减起点，验有限且非负，然后入队并裁窗。
bool LatencyEstimator::recordFrame(const FrameTiming& timing) {
  // 作用：算这条流水线一共花了多久。实现：命令就绪时间减去曝光开始时间。
  const double latency = timing.command_ready_time - timing.exposure_time;
  // 作用：挡住时间倒流和 NaN，坏数据不进统计。实现：isfinite 加非负判断，不合格直接退货。
  if (!std::isfinite(latency) || latency < 0.0) return false;

  // 作用：入队前先上锁。实现：lock_guard 守到函数结束。
  std::lock_guard<std::mutex> lock(mutex_);
  // 作用：新样本入账。实现：push_back 到队尾。
  process_latencies_.push_back(latency);
  // 作用：窗口超员就丢最老的，保持滑窗。实现：while 循环 pop_front。
  while (process_latencies_.size() > window_size_) {
    process_latencies_.pop_front();
  }
  return true;
}

// 作用：记录通信段的耗时。实现：先验数，再入队，最后削到窗口大小。
void LatencyEstimator::recordCommunication(double seconds) {
  // 作用：过滤负数和非有限值，通信时间没道理为负。实现：isfinite 加非负判断。
  if (!std::isfinite(seconds) || seconds < 0.0) return;

  std::lock_guard<std::mutex> lock(mutex_);
  communication_latencies_.push_back(seconds);
  // 作用：维持窗口规模，老样本让位。实现：while 循环从队头弹出。
  while (communication_latencies_.size() > window_size_) {
    communication_latencies_.pop_front();
  }
}

// 作用：记录弹丸飞行时间。实现：和通信延迟同一套流程，只是换个队列。
void LatencyEstimator::recordFlightTime(double seconds) {
  // 作用：飞行时间同样不接受负数和 NaN。实现：isfinite 加非负判断。
  if (!std::isfinite(seconds) || seconds < 0.0) return;

  std::lock_guard<std::mutex> lock(mutex_);
  flight_times_.push_back(seconds);
  // 作用：裁到窗口上限，防止历史无限长。实现：while 循环 pop_front。
  while (flight_times_.size() > window_size_) {
    flight_times_.pop_front();
  }
}

// 作用：打包一份三类延迟的统计结果。实现：锁内分别汇总三个队列，再判断能否拼出总延迟。
LatencySnapshot LatencyEstimator::snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);

  // 作用：准备一张空报表。实现：默认构造，字段全零、valid 为假。
  LatencySnapshot out;
  // 作用：处理段统计。实现：调用 summarize 压成三件套。
  out.process = summarize(process_latencies_);
  // 作用：通信段统计。实现：同上。
  out.communication = summarize(communication_latencies_);
  // 作用：飞行段统计。实现：同上。
  out.flight = summarize(flight_times_);

  // 作用：判定总延迟有没有资格存在。实现：三段样本数都大于 0 才置 valid。
  out.valid = out.process.sample_count > 0 && out.communication.sample_count > 0 && out.flight.sample_count > 0;

  // 作用：三段齐全才求和，缺一段就别报总账。实现：中位数相加，比均值更抗个别极端样本。
  if (out.valid) {
    out.total_delay = out.process.median + out.communication.median + out.flight.median;
  }

  return out;
}

// 作用：清空统计，换场景或重新标定时用。实现：锁内把三个队列一起清掉。
void LatencyEstimator::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  process_latencies_.clear();
  communication_latencies_.clear();
  flight_times_.clear();
}

// 作用：把一堆延迟样本浓缩成均值、中位、最大值。实现：空窗直接返回零报表，否则累加求和、扫最大值、nth_element 求中位。
LatencyStats LatencyEstimator::summarize(const std::deque<double>& values) {
  // 作用：准备一张空统计表。实现：默认构造，全零。
  LatencyStats stats;
  // 作用：没样本就别硬算，免得除以零。实现：empty 早退。
  if (values.empty()) return stats;

  // 作用：记录样本数，同时也是后面除法的分母。实现：直接取 size。
  stats.sample_count = values.size();
  // 作用：算平均值，看整体趋势。实现：accumulate 求和后除以样本数。
  stats.mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  // 作用：抓最坏情况，排查抖动源头。实现：max_element 扫一遍取最大。
  stats.maximum = *std::max_element(values.begin(), values.end());

  // 作用：为了求中位数先拷一份，别动原队列顺序。实现：用 vector 区间构造。
  std::vector<double> sorted(values.begin(), values.end());
  // 作用：定位中位数下标。实现：begin 加 size/2，偶数个取偏右。
  const auto middle = sorted.begin() + static_cast<std::ptrdiff_t>(sorted.size() / 2);
  // 作用：只保证中位数落位，不做全排序，省时省力。实现：nth_element，平均 O(n)。
  std::nth_element(sorted.begin(), middle, sorted.end());
  // 作用：落袋为安。实现：解引用 middle 赋给 median。
  stats.median = *middle;
  return stats;
}

}  // namespace kalfilter
