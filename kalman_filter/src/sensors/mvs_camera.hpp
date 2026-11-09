# ifndef AUTOAIM_MVS_CAMERA_HPP
# define AUTOAIM_MVS_CAMERA_HPP

# include <cstdint>
# include <string>
# include <vector>

# include <opencv2/core.hpp>

# include "core/config.hpp"
# include "sensors/camera_source.hpp"

namespace camera
{

using MvsCameraConfig = core::CameraConfig;

struct CameraInfo
{
    unsigned int index {};
    unsigned int transport_layer {};
    std::string model;
    std::string serial;
    std::string user_name;
};

using CameraFrame = core::TimestampedFrame;

class MvsCamera final : public CameraSource
{
public:
    explicit MvsCamera(const MvsCameraConfig& config = {});
    ~MvsCamera() override;

    MvsCamera(const MvsCamera&) = delete;
    MvsCamera& operator=(const MvsCamera&) = delete;

    static std::vector<CameraInfo> enumerate(std::string* error = nullptr);

    bool open(std::string* error = nullptr) override;
    core::FrameReadStatus read(
        CameraFrame& frame,
        std::string* error = nullptr) override;
    void close() override;

    bool isOpen() const override { return handle_ != nullptr; }
    SourceInfo info() const override { return info_; }

private:
    bool fail(const std::string& message, std::string* error);
    bool applyConfig(
        const MvsCameraConfig& config,
        std::string* error);

    MvsCameraConfig config_;
    void* handle_ {};
    bool grabbing_ {};
    bool sdk_acquired_ {};
    SourceInfo info_;
    std::string last_error_;
};

}   // namespace camera

# endif
