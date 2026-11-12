# ifndef AUTOAIM_LATENCY_ESTIMATOR_HPP
# define AUTOAIM_LATENCY_ESTIMATOR_HPP

# include <cstddef>
# include <deque>
# include <mutex>

namespace kalfilter
{

struct FrameTiming
{
    // 作用：曝光开始的时刻，整条延迟链的起点。实现：上位机秒数，recordFrame 拿它当被减数。
    double exposure_time {};
    // 作用：图像到手时刻，可用来单独看采集延迟。实现：只记录不参与当前计算，留着以后扩展。
    double frame_received_time {};
    // 作用：检测跑完的时刻。实现：同上，先当观察点存着。
    double detection_done_time {};
    // 作用：PnP 解算完成的时刻。实现：同上，分段打点方便定位瓶颈。
    double pnp_done_time {};
    // 作用：EKF 更新完成的时刻。实现：同上，逐段留痕。
    double ekf_done_time {};
    // 作用：控制命令打包就绪的时刻，延迟链的终点。实现：recordFrame 用它减曝光时间。
    double command_ready_time {};
};

struct LatencyStats
{
    // 作用：这组统计用了几个样本。实现：取窗口长度，空窗时为 0。
    std::size_t sample_count {};
    // 作用：平均值，看整体水平。实现：总和除以个数。
    double mean {};
    // 作用：中位数，抗毛刺的主力指标。实现：nth_element 取中间。
    double median {};
    // 作用：最大值，看最坏情况有多坏。实现：max_element 扫一遍。
    double maximum {};
};

struct LatencySnapshot
{
    // 作用：处理段统计，曝光到命令就绪。实现：由 process_latencies_ 汇总而来。
    LatencyStats process;
    // 作用：通信段统计，命令发出去的耗时。实现：由 communication_latencies_ 汇总。
    LatencyStats communication;
    // 作用：飞行段统计，弹丸飞到目标的时间。实现：由 flight_times_ 汇总。
    LatencyStats flight;
    // 作用：三段中位数之和，理论总延迟。实现：只有三段都有样本时才由 snapshot 填值。
    double total_delay {};
    // 作用：标志这份快照可不可信。实现：三段样本数都大于 0 才置 true。
    bool valid {};
};

/*
 * process_latency 从曝光开始，到命令准备完成。
 * comm_latency 和 fly_time 分别单独记录；total_delay 只在三者都有有效值时成立。
 */
class LatencyEstimator
{
public:
    // 作用：定下统计窗口，避免样本无限增长。实现：窗口至少给 1，非正数也被夹回来。
    explicit LatencyEstimator(std::size_t window_size = 120);

    // 作用：交一帧的完整打点，换一条处理延迟入账。实现：终点减起点，验非负后入队裁窗。
    bool recordFrame(const FrameTiming& timing);
    // 作用：单独记一次通信延迟，由通信模块上报。实现：验数后推进队列，超窗就丢老的。
    void recordCommunication(double seconds);
    // 作用：单独记一次弹丸飞行时间，由弹道模型算出。实现：同通信延迟，验数、入队、裁窗。
    void recordFlightTime(double seconds);

    // 作用：导出一份当前统计快照给 UI 或日志。实现：锁内汇总三组队列，三段齐全才算总延迟。
    LatencySnapshot snapshot() const;
    // 作用：清空所有样本，重新开始统计。实现：锁内清三个 deque。
    void clear();

private:
    // 作用：把一串样本压成均值/中位/最大值三件套。实现：累加、max_element、nth_element 各来一次。
    static LatencyStats summarize(const std::deque<double>& values);

    // 作用：窗口容量，三类延迟共用同一个上限。实现：构造时夹到至少 1，入队时判 size。
    std::size_t window_size_ {};
    // 作用：保护三个队列，因为上报线程和查询线程各干各的。实现：所有入口先 lock_guard。
    mutable std::mutex mutex_;
    // 作用：处理延迟样本，单位秒。实现：deque 存，满了从队头 pop。
    std::deque<double> process_latencies_;
    // 作用：通信延迟样本，单位秒。实现：同为 deque，独立计数。
    std::deque<double> communication_latencies_;
    // 作用：飞行时间样本，单位秒。实现：同为 deque，单独维护。
    std::deque<double> flight_times_;
};

}   // namespace kalfilter

# endif
