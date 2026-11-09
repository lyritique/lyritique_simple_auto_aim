# ifndef AUTOAIM_CORE_FRAME_HPP
# define AUTOAIM_CORE_FRAME_HPP

# include <cstdint>

# include <opencv2/core.hpp>

namespace core
{

enum class TimeDomain
{
    HostMonotonic,
    VideoRelative
};

enum class FrameReadStatus
{
    Ok,
    Timeout,
    End,
    Error
};

struct TimestampedFrame
{
    cv::Mat bgr;

    double timestamp {};
    TimeDomain time_domain {TimeDomain::HostMonotonic};

    uint64_t device_timestamp {};
    int64_t sdk_host_timestamp {};
    uint64_t frame_number {};
    float exposure_time_us {};
};

}   // namespace core

# endif
