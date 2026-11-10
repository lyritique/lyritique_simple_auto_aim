# ifndef AUTOAIM_CORE_FRAME_HPP
# define AUTOAIM_CORE_FRAME_HPP

# include <cstdint>

# include <opencv2/core.hpp>

namespace core
{

////在目前的代码中毫无用处
enum class TimeDomain
{
    HostMonotonic,
    VideoRelative
};

//相机源接口 CameraSource::read() 的返回值
enum class FrameReadStatus
{
    Ok,     //拿到一帧
    Timeout,//这轮没数据
    End,    //视频读完
    Error   //出错
};

struct TimestampedFrame
{
    cv::Mat bgr;

    double timestamp {};
    //在目前的代码中毫无用处
    TimeDomain time_domain {TimeDomain::HostMonotonic};

    uint64_t device_timestamp {}; //无人读
    int64_t sdk_host_timestamp {};//无人读
    uint64_t frame_number {};
    float exposure_time_us {};    //无人读
};

}   // namespace core

# endif
