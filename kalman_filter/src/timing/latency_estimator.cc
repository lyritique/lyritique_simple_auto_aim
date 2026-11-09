# include "timing/latency_estimator.hpp"

# include <algorithm>
# include <cmath>
# include <numeric>
# include <vector>

namespace kalfilter
{

LatencyEstimator::LatencyEstimator(std::size_t window_size)
    : window_size_(std::max<std::size_t>(1, window_size))
{
}

bool LatencyEstimator::recordFrame(const FrameTiming& timing)
{
    const double latency =
        timing.command_ready_time - timing.exposure_time;
    if (!std::isfinite(latency) || latency < 0.0) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    process_latencies_.push_back(latency);
    while (process_latencies_.size() > window_size_)
    {
        process_latencies_.pop_front();
    }
    return true;
}

void LatencyEstimator::recordCommunication(double seconds)
{
    if (!std::isfinite(seconds) || seconds < 0.0) return;

    std::lock_guard<std::mutex> lock(mutex_);
    communication_latencies_.push_back(seconds);
    while (communication_latencies_.size() > window_size_)
    {
        communication_latencies_.pop_front();
    }
}

void LatencyEstimator::recordFlightTime(double seconds)
{
    if (!std::isfinite(seconds) || seconds < 0.0) return;

    std::lock_guard<std::mutex> lock(mutex_);
    flight_times_.push_back(seconds);
    while (flight_times_.size() > window_size_)
    {
        flight_times_.pop_front();
    }
}

LatencySnapshot LatencyEstimator::snapshot() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    LatencySnapshot out;
    out.process = summarize(process_latencies_);
    out.communication = summarize(communication_latencies_);
    out.flight = summarize(flight_times_);

    out.valid =
        out.process.sample_count > 0 &&
        out.communication.sample_count > 0 &&
        out.flight.sample_count > 0;

    if (out.valid)
    {
        out.total_delay =
            out.process.median +
            out.communication.median +
            out.flight.median;
    }

    return out;
}

void LatencyEstimator::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    process_latencies_.clear();
    communication_latencies_.clear();
    flight_times_.clear();
}

LatencyStats LatencyEstimator::summarize(
    const std::deque<double>& values)
{
    LatencyStats stats;
    if (values.empty()) return stats;

    stats.sample_count = values.size();
    stats.mean =
        std::accumulate(values.begin(), values.end(), 0.0) /
        static_cast<double>(values.size());
    stats.maximum = *std::max_element(values.begin(), values.end());

    std::vector<double> sorted(values.begin(), values.end());
    const auto middle = sorted.begin() +
        static_cast<std::ptrdiff_t>(sorted.size() / 2);
    std::nth_element(sorted.begin(), middle, sorted.end());
    stats.median = *middle;
    return stats;
}

}   // namespace kalfilter
