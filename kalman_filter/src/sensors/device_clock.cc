#include "sensors/device_clock.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace camera {
namespace {

// 中位数。入参按值传，允许就地 nth_element 打乱。
double median(std::vector<double> values) {
  if (values.empty()) return 0.0;
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  return values[mid];
}

}  // namespace

void DeviceClock::reset(double scale, double offset) {
  samples_.clear();
  scale_ = scale;
  offset_ = offset;
  target_scale_ = scale;
  have_fit_ = false;
  bad_fits_ = 0;
  model_broken_ = false;
}

void DeviceClock::push(std::uint64_t ticks, double host) {
  // 时间必须往前走，倒退的样本是回绕或校准切换，直接丢，别把窗口搅乱。
  if (!samples_.empty() && ticks <= samples_.back().ticks) return;
  if (!std::isfinite(host)) return;

  samples_.push_back(Sample{ticks, host});
  while (samples_.size() > options_.capacity) samples_.pop_front();
}

bool DeviceClock::fitLine(const Sample* pts, std::size_t count, double& slope, double& intercept) {
  if (pts == nullptr || count < 2) return false;

  double mean_x = 0.0;
  double mean_y = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    mean_x += static_cast<double>(pts[i].ticks);
    mean_y += pts[i].host;
  }
  mean_x /= static_cast<double>(count);
  mean_y /= static_cast<double>(count);

  double sxx = 0.0;
  double sxy = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const double dx = static_cast<double>(pts[i].ticks) - mean_x;
    const double dy = pts[i].host - mean_y;
    sxx += dx * dx;
    sxy += dx * dy;
  }
  if (!(sxx > 0.0)) return false;

  slope = sxy / sxx;
  intercept = mean_y - slope * mean_x;
  return std::isfinite(slope) && std::isfinite(intercept);
}

DeviceClock::Report DeviceClock::update(std::uint64_t now_ticks, double lag_s) {
  Report report;
  report.samples = samples_.size();
  report.scale = scale_;
  report.model_broken = model_broken_;

  // 窗口还没攒够、或者现在正积压：都不动。积压时收帧时刻里混着排队时间，
  // 拟合出来的是「到达速率」而不是「曝光速率」。
  if (samples_.size() < options_.min_samples) return report;
  if (lag_s > options_.freeze_lag_s) return report;

  std::vector<Sample> pts(samples_.begin(), samples_.end());

  const double span_s =
      static_cast<double>(pts.back().ticks - pts.front().ticks) * (scale_ > 0.0 ? scale_ : 1e-8);
  report.span_s = span_s;
  if (span_s < options_.min_span_s) return report;

  double slope = 0.0;
  double intercept = 0.0;
  if (!fitLine(pts.data(), pts.size(), slope, intercept)) return report;

  // 残差分布是单边偏的（卡顿只会让收帧变晚），用 MAD 而不是标准差来剔离群。
  std::vector<double> residual(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    residual[i] = pts[i].host - (slope * static_cast<double>(pts[i].ticks) + intercept);
  }
  const double center = median(residual);

  std::vector<double> abs_dev(pts.size());
  for (std::size_t i = 0; i < residual.size(); ++i) abs_dev[i] = std::abs(residual[i] - center);
  const double mad = median(abs_dev) * 1.4826;

  // 下限 0.2 ms：正常收帧抖动就有一两毫秒，别把它全当离群砍掉。
  const double limit = std::max(3.5 * mad, 2e-4);

  std::vector<Sample> kept;
  kept.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (std::abs(residual[i] - center) <= limit) kept.push_back(pts[i]);
  }
  if (kept.size() < options_.min_samples / 2) return report;

  if (!fitLine(kept.data(), kept.size(), slope, intercept)) return report;

  double sum_sq = 0.0;
  for (const auto& s : kept) {
    const double err = s.host - (slope * static_cast<double>(s.ticks) + intercept);
    sum_sq += err * err;
  }
  const double rms_ms = std::sqrt(sum_sq / static_cast<double>(kept.size())) * 1000.0;
  report.residual_ms = rms_ms;
  report.fit_scale = slope;

  // 残差太大：戳和收帧时刻根本不成线性关系，说明模型不成立（真积压、
  // 或者换了个不是名义时钟的相机）。连续几次就判死，让调用方退回主机时间。
  if (rms_ms > options_.max_residual_ms) {
    if (++bad_fits_ >= options_.max_bad_fits) model_broken_ = true;
    report.model_broken = model_broken_;
    return report;
  }
  bad_fits_ = 0;

  if (slope < options_.scale_lo || slope > options_.scale_hi) return report;
  // 第一次不设限：先验 10 ns/tick 可能和真值差 7%，挡住了就永远收敛不了。
  // 之后拿上一次被接受的拟合值比，而不是拿平滑后的 scale_ —— 否则平滑一步之后
  // 剩余偏差还在 7% 量级，护栏会把后续每一次更新全判掉。
  if (have_fit_ && std::abs(slope / target_scale_ - 1.0) > options_.max_scale_jump) return report;
  target_scale_ = slope;

  // 平滑后重锚：换斜率会让 stamp 整体平移 now_ticks×(旧-新)，
  // 把这段补进 offset，当前帧的映射就严丝合缝地接上，不会跳。
  const double next_scale = scale_ + options_.smooth * (slope - scale_);
  offset_ += static_cast<double>(now_ticks) * (scale_ - next_scale);
  scale_ = next_scale;

  // 截距单独拉：传输延迟只会让 host 变大，所以窗口里 (host - scale*ticks) 的最小值
  // 最接近真实曝光时刻。限速挪，避免 stamp 抖动。
  double best = std::numeric_limits<double>::infinity();
  for (const auto& s : kept) {
    best = std::min(best, s.host - scale_ * static_cast<double>(s.ticks));
  }
  if (std::isfinite(best)) {
    const double step =
        std::clamp(best - offset_, -options_.max_offset_step_s, options_.max_offset_step_s);
    offset_ += step;
  }

  have_fit_ = true;
  report.updated = true;
  report.scale = scale_;
  return report;
}

}  // namespace camera
