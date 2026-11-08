# ifndef AUTOAIM_GIM_HISTORY_
# define AUTOAIM_GIM_HISTORY_

# include <cstddef>
# include <cstdint>
# include <deque>
# include <mutex>

# include <eigen3/Eigen/Geometry>

namespace kalfilter
{

/*
 * 一条 C 板反馈。host_time 使用上位机的 steady_clock 秒数；
 * q 已经由 C 板换算成“相机系 -> 参考世界系”，上位机不再乘安装矩阵。
 */
struct GimbalSample
{
    double host_time {};
    uint64_t board_time_us {};

    Eigen::Quaterniond q {Eigen::Quaterniond::Identity()};

    float pitch {};
    float pitch_vel {};
    float yaw {};
    float yaw_vel {};
    float bullet_speed {};

    uint8_t mode {};
};

/*
 * 按时间维护最近的云台反馈。
 *
 * interpolate() 只接受被两个历史样本夹住的时间，绝不为了图像对齐而外推。
 * predict() 只用于未来控制预测，并且限制外推时长。
 */
class GimbalFeedbackHistory
{
public:
    explicit GimbalFeedbackHistory(
        double max_history_seconds = 2.0,
        double max_interpolation_gap_seconds = 0.1,
        double max_prediction_seconds = 0.05);

    // 时间必须严格递增；重复或乱序样本返回 false。
    bool push(const GimbalSample& sample);
    void clear();

    // 在 [a.host_time, b.host_time] 内 SLERP/线性插值；区间外返回 false。
    bool interpolate(double host_time, GimbalSample& out) const;

    // 返回当前历史中的最新有效样本。
    bool latest(GimbalSample& out) const;

    // 历史区间内调用 interpolate；未来最多外推 max_prediction_seconds。
    bool predict(double host_time, GimbalSample& out) const;

    bool empty() const;
    std::size_t size() const;

private:
    bool interpolateUnlocked(double host_time, GimbalSample& out) const;
    bool predictUnlocked(double host_time, GimbalSample& out) const;
    void pruneUnlocked();

    double max_history_seconds_ {};
    double max_interpolation_gap_seconds_ {};
    double max_prediction_seconds_ {};

    mutable std::mutex mutex_;
    std::deque<GimbalSample> samples_;
};

}   // namespace kalfilter

# endif
