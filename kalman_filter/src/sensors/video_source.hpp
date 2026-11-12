# ifndef AUTOAIM_SENSORS_VIDEO_SOURCE_HPP
# define AUTOAIM_SENSORS_VIDEO_SOURCE_HPP

# include <string>

# include <opencv2/videoio.hpp>

# include "sensors/camera_source.hpp"

namespace camera
{

class VideoSource final : public CameraSource
{
public:
    explicit VideoSource(std::string path);

    bool open(std::string* error = nullptr) override;
    core::FrameReadStatus read(
        core::TimestampedFrame& frame,
        std::string* error = nullptr) override;
    void close() override;

    bool isOpen() const override;
    SourceInfo info() const override;

private:
    std::string path_;
    cv::VideoCapture capture_;
    SourceInfo info_;
    uint64_t next_frame_number_ {};
};

}   // namespace camera

# endif
