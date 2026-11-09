# ifndef AUTOAIM_TIMING_REPLAY_IMU_HPP
# define AUTOAIM_TIMING_REPLAY_IMU_HPP

# include <string>
# include <vector>

# include <Eigen/Geometry>

namespace replay
{

struct ImuSample
{
    double time {};
    Eigen::Quaterniond q {Eigen::Quaterniond::Identity()};
};

std::vector<ImuSample> readImuFile(const std::string& path);

ImuSample interpolateImu(
    const std::vector<ImuSample>& samples,
    double image_time);

ImuSample alignImuToImage(
    const std::vector<ImuSample>& samples,
    double image_time,
    double offset_seconds = 0.009);

}   // namespace replay

# endif
