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
    // 作用：给同步器定个出厂设置。实现：延迟负数一律按 0 处理，窗口至少留 1 个格子，省得后面空窗或除零。
    explicit ClockSync(
        double comm_latency_seconds = 0.0,
        std::size_t max_observations = 128);

    // 作用：运行中改通信延迟（比如刚标定出新值）。实现：加锁改一个 double，负值同样被夹回 0。
    void setCommLatency(double seconds);
    // 作用：把当前延迟借出去给日志或 UI 看。实现：加锁读一份副本，避免看到改到一半的值。
    double commLatency() const;

    // 作用：一次调用干三件事——展开回绕、验单调、算时间偏移。实现：校验后进锁，32 位戳补成 64 位，再用中位数更新 offset。
    bool observeAndMap(
        float board_stamp_us,
        double receive_host_time,
        uint64_t& unwrapped_board_us,
        double& event_host_time);

    //棍木
    // 作用：手里已有展开后的板卡时间，只想要上位机时间。实现：乘 1e-6 换成秒，加上当前 offset，没 offset 就老实返回 false。
    bool boardToHost(
        uint64_t unwrapped_board_us,
        double& event_host_time) const;

    // 作用：问一句“现在能不能换算”。实现：加锁读 has_offset_，它只在中位数算出来后才为真。
    bool hasOffset() const;
    // 作用：清空所有记忆，假装第一次见到 C 板。实现：锁内把开关、计数器、队列、offset 一起归零。
    void reset();

private:
    // 作用：把新观测塞进窗口并重算中位数 offset。实现：deque 满了从队头踢人，然后 nth_element 找中间值，O(n) 不必全排序。
    void updateOffsetUnlocked(double observation);

    // 作用：保护下面所有状态，因为喂数据和查时间多半不在一个线程。实现：所有 public 入口先 lock_guard 再说。
    mutable std::mutex mutex_;

    // 作用：通信延迟（秒），把“包到我这的时间”倒推回“事件发生的时间”。实现：一个标量，构造和设置时都夹到非负。
    double comm_latency_seconds_ {};
    // 作用：offset 中位数窗口的大小，防止历史无限膨胀。实现：只是一个上限数字，由 updateOffsetUnlocked 执行裁员。
    std::size_t max_observations_ {};

    // 作用：记号笔，记着“见过第一个原始戳没”。实现：bool 开关，首次观察时置 true 并立基准。
    bool has_raw_ {};
    // 作用：记号笔，记着“有没有展开后的时间”。实现：bool 开关，给单调性检查当门禁。
    bool has_unwrapped_ {};
    // 作用：上一个 32 位原始微秒值，用来判断这次是不是跨过回绕点。实现：直接存上一次 raw，新老一比就知道。
    std::uint32_t last_raw_us_ {};
    // 作用：累计回绕补偿，跨一次圈就加 2^32 微秒。实现：64 位累加器，只增不减，时间因此单调向前。
    std::uint64_t wrap_offset_us_ {};
    // 作用：上一个展开后的时间戳，充当交通警察。实现：64 位保存，新候选值必须严格更大才放行。
    std::uint64_t last_unwrapped_us_ {};

    // 作用：最近若干次“到达时间 - 板卡时间”的差值，是 offset 的原料。实现：双端队列，超窗就从队头丢。
    std::deque<double> offsets_;
    // 作用：当前使用的板卡到上位机时间偏移（秒）。实现：对 offsets_ 取中位数，单次毛刺撼不动它。
    double offset_seconds_ {};
    // 作用：offset 的营业执照，没它就别做换算。实现：成功算出一次中位数就置 true，reset 再吊销。
    bool has_offset_ {};
};

}   // namespace kalfilter

# endif
