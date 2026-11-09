# include "timing/clock_sync.hpp"

# include <algorithm>
# include <cmath>
# include <limits>
# include <vector>

namespace kalfilter
{
namespace
{

constexpr double kMicrosecond = 1e-6;
constexpr std::uint64_t kUint32Range = 1ULL << 32;
constexpr std::uint64_t kHalfUint32Range = 1ULL << 31;

}   // namespace

ClockSync::ClockSync(
    double comm_latency_seconds,
    std::size_t max_observations)
    : comm_latency_seconds_(
          std::max(0.0, comm_latency_seconds)),
      max_observations_(
          std::max<std::size_t>(1, max_observations))
{
}

void ClockSync::setCommLatency(double seconds)
{
    std::lock_guard<std::mutex> lock(mutex_);
    comm_latency_seconds_ = std::max(0.0, seconds);
}

double ClockSync::commLatency() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return comm_latency_seconds_;
}

bool ClockSync::observeAndMap(
    float board_stamp_us,
    double receive_host_time,
    uint64_t& unwrapped_board_us,
    double& event_host_time)
{
    if (!std::isfinite(board_stamp_us) ||
        board_stamp_us < 0.0F ||
        !std::isfinite(receive_host_time))
    {
        return false;
    }

    const auto rounded = std::llround(board_stamp_us);
    const auto max_plus_one =
        static_cast<long long>(kUint32Range);
    if (rounded < 0 || rounded > max_plus_one)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    const std::uint32_t raw =
        rounded == max_plus_one
            ? 0
            : static_cast<std::uint32_t>(rounded);

    if (!has_raw_)
    {
        has_raw_ = true;
        last_raw_us_ = raw;
        wrap_offset_us_ = 0;
    }
    else if (raw < last_raw_us_ &&
             static_cast<std::uint64_t>(last_raw_us_ - raw) >
                 kHalfUint32Range)
    {
        wrap_offset_us_ += kUint32Range;
    }

    const std::uint64_t candidate =
        wrap_offset_us_ + static_cast<std::uint64_t>(raw);

    // 乱序或重复的 C 板时间戳不能写进单调历史。
    if (has_unwrapped_ && candidate <= last_unwrapped_us_) return false;

    unwrapped_board_us = candidate;
    last_raw_us_ = raw;
    last_unwrapped_us_ = candidate;
    has_unwrapped_ = true;

    const double observation =
        (receive_host_time - comm_latency_seconds_) -
        static_cast<double>(candidate) * kMicrosecond;
    updateOffsetUnlocked(observation);

    event_host_time =
        static_cast<double>(candidate) * kMicrosecond +
        offset_seconds_;
    return true;
}

bool ClockSync::boardToHost(
    uint64_t unwrapped_board_us,
    double& event_host_time) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_offset_) return false;

    event_host_time =
        static_cast<double>(unwrapped_board_us) * kMicrosecond +
        offset_seconds_;
    return true;
}

bool ClockSync::hasOffset() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return has_offset_;
}

void ClockSync::reset()
{
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

void ClockSync::updateOffsetUnlocked(double observation)
{
    offsets_.push_back(observation);
    while (offsets_.size() > max_observations_)
    {
        offsets_.pop_front();
    }

    std::vector<double> sorted(offsets_.begin(), offsets_.end());
    const auto middle = sorted.begin() +
        static_cast<std::ptrdiff_t>(sorted.size() / 2);
    std::nth_element(sorted.begin(), middle, sorted.end());
    offset_seconds_ = *middle;
    has_offset_ = true;
}

}   // namespace kalfilter
