# include "sensors/video_source.hpp"

# include <utility>

namespace camera
{

VideoSource::VideoSource(std::string path)
    : path_(std::move(path))
{
}

bool VideoSource::open(std::string* error)
{
    close();

    if (!capture_.open(path_))
    {
        if (error) *error = "读不了视频文件: " + path_;
        return false;
    }

    const double frame_rate =
        capture_.get(cv::CAP_PROP_FPS);
    const double frame_count =
        capture_.get(cv::CAP_PROP_FRAME_COUNT);
    if (frame_rate <= 0.0 || frame_rate > 1000.0)
    {
        if (error) *error = "视频帧率无效: " + path_;
        close();
        return false;
    }

    cv::Mat first;
    capture_ >> first;
    if (first.empty())
    {
        if (error) *error = "视频第一帧读不了: " + path_;
        close();
        return false;
    }

    info_.width = first.cols;
    info_.height = first.rows;
    info_.frame_rate = frame_rate;
    info_.frame_count = frame_count > 0.0
        ? static_cast<uint64_t>(frame_count)
        : 0;

    capture_.set(cv::CAP_PROP_POS_FRAMES, 0);
    next_frame_number_ = 0;
    return true;
}

core::FrameReadStatus VideoSource::read(
    core::TimestampedFrame& frame,
    std::string* error)
{
    frame = core::TimestampedFrame {};
    if (!capture_.isOpened())
    {
        if (error) *error = "视频尚未打开";
        return core::FrameReadStatus::Error;
    }

    capture_ >> frame.bgr;
    if (frame.bgr.empty())
    {
        return core::FrameReadStatus::End;
    }

    frame.frame_number = next_frame_number_;
    frame.timestamp =
        static_cast<double>(next_frame_number_) / info_.frame_rate;
    frame.time_domain = core::TimeDomain::VideoRelative;
    ++next_frame_number_;
    return core::FrameReadStatus::Ok;
}

void VideoSource::close()
{
    if (capture_.isOpened()) capture_.release();
    info_ = SourceInfo {};
    next_frame_number_ = 0;
}

bool VideoSource::isOpen() const
{
    return capture_.isOpened();
}

SourceInfo VideoSource::info() const
{
    return info_;
}

}   // namespace camera
