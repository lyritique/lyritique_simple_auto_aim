# ifndef AUTOAIM_TIMING_REPLAY_IMU_HPP
# define AUTOAIM_TIMING_REPLAY_IMU_HPP

# include <string>
# include <vector>

# include <Eigen/Geometry>

namespace replay
{

struct ImuSample
{
    // 作用：这帧 IMU 的时间，插值的排序依据。实现：double 秒，要求输入文件里已按时间递增。
    double time {};
    // 作用：该时刻的姿态四元数，回放时喂给 EKF。实现：Eigen 四元数，默认单位旋转当占位符。
    Eigen::Quaterniond q {Eigen::Quaterniond::Identity()};
};

// 作用：把 IMU 文本文件读成样本数组。实现：逐行解析，五列数值全读到才收，坏行打日志跳过。
std::vector<ImuSample> readImuFile(const std::string& path);

// 作用：在 IMU 序列里插出任意时刻的姿态。实现：lower_bound 找右邻居，左右两端 SLERP，越界就贴边返回端点。
ImuSample interpolateImu(
    const std::vector<ImuSample>& samples,
    double image_time);

// 作用：补偿一个固定时间差后，把 IMU 对齐到图像时刻。实现：把图像时间减掉 offset 再转调插值。
ImuSample alignImuToImage(
    const std::vector<ImuSample>& samples,
    double image_time,
    double offset_seconds = 0.009);

}   // namespace replay

# endif
