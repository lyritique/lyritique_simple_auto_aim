# ifndef AUTOAIM_LATENCY_ESTIMATOR_HPP
# define AUTOAIM_LATENCY_ESTIMATOR_HPP

# include <cstddef>
# include <deque>
# include <mutex>

namespace kalfilter
{

struct FrameTiming
{
    double exposure_time {};
    double frame_received_time {};
    double detection_done_time {};
    double pnp_done_time {};
    double ekf_done_time {};
    double command_ready_time {};
};

struct LatencyStats
{
    std::size_t sample_count {};
    double mean {};
    double median {};
    double maximum {};
};

struct LatencySnapshot
{
    LatencyStats process;
    LatencyStats communication;
    LatencyStats flight;
    double total_delay {};
    bool valid {};
};

/*
 * process_latency 从曝光开始，到命令准备完成。
 * comm_latency 和 fly_time 分别单独记录；total_delay 只在三者都有有效值时成立。
 */
class LatencyEstimator
{
public:
    explicit LatencyEstimator(std::size_t window_size = 120);

    bool recordFrame(const FrameTiming& timing);
    void recordCommunication(double seconds);
    void recordFlightTime(double seconds);

    LatencySnapshot snapshot() const;
    void clear();

private:
    static LatencyStats summarize(const std::deque<double>& values);

    std::size_t window_size_ {};
    mutable std::mutex mutex_;
    std::deque<double> process_latencies_;
    std::deque<double> communication_latencies_;
    std::deque<double> flight_times_;
};

}   // namespace kalfilter

# endif
