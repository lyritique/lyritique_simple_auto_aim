# ifndef AUTOAIM_CLOCK_SYNC_HPP
# define AUTOAIM_CLOCK_SYNC_HPP

# include <cstddef>
# include <cstdint>
# include <deque>
# include <mutex>

namespace kalfilter
{

/*
 * 把 C 板的 DWT 微秒时间映射到上位机 steady_clock 秒。
 *
 * DWT_stamp 的浮点数量级可能很大，且 32 位微秒计数约 71.6 分钟回绕一次。
 * observeAndMap() 负责展开回绕；通信延迟只用于修正“时间戳事件 -> 包到达”的偏差。
 */
class ClockSync
{
public:
    explicit ClockSync(
        double comm_latency_seconds = 0.0,
        std::size_t max_observations = 128);

    void setCommLatency(double seconds);
    double commLatency() const;

    bool observeAndMap(
        float board_stamp_us,
        double receive_host_time,
        uint64_t& unwrapped_board_us,
        double& event_host_time);

    bool boardToHost(
        uint64_t unwrapped_board_us,
        double& event_host_time) const;

    bool hasOffset() const;
    void reset();

private:
    void updateOffsetUnlocked(double observation);

    mutable std::mutex mutex_;

    double comm_latency_seconds_ {};
    std::size_t max_observations_ {};

    bool has_raw_ {};
    bool has_unwrapped_ {};
    std::uint32_t last_raw_us_ {};
    std::uint64_t wrap_offset_us_ {};
    std::uint64_t last_unwrapped_us_ {};

    std::deque<double> offsets_;
    double offset_seconds_ {};
    bool has_offset_ {};
};

}   // namespace kalfilter

# endif
