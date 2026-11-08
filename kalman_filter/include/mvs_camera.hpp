# ifndef AUTOAIM_MVS_CAMERA_HPP
# define AUTOAIM_MVS_CAMERA_HPP

# include <cstdint>
# include <string>
# include <vector>

# include <opencv2/core.hpp>

namespace camera
{

struct MvsCameraConfig
{
    unsigned int device_index {};
    int timeout_ms {1000};

    // 0 表示不主动修改，保留相机当前值。
    int width {};
    int height {};
    float frame_rate {};
    float exposure_time_us {};
    float gain {-1.0F};
};

struct CameraInfo
{
    unsigned int index {};
    unsigned int transport_layer {};
    std::string model;
    std::string serial;
    std::string user_name;
};

struct CameraFrame
{
    cv::Mat bgr;

    // 上位机在 MV_CC_GetImageBuffer 返回后的 steady_clock 秒数。
    double host_time {};

    uint64_t device_timestamp {};
    int64_t sdk_host_timestamp {};
    uint64_t frame_number {};
    float exposure_time_us {};
};

class MvsCamera
{
public:
    MvsCamera() = default;
    ~MvsCamera();

    MvsCamera(const MvsCamera&) = delete;
    MvsCamera& operator=(const MvsCamera&) = delete;

    static std::vector<CameraInfo> enumerate(std::string* error = nullptr);

    bool open(
        const MvsCameraConfig& config,
        std::string* error = nullptr);
    bool read(CameraFrame& frame, std::string* error = nullptr);
    void close();

    bool isOpen() const noexcept { return handle_ != nullptr; }

private:
    bool fail(const std::string& message, std::string* error);
    bool applyConfig(
        const MvsCameraConfig& config,
        std::string* error);

    void* handle_ {};
    bool grabbing_ {};
    bool sdk_acquired_ {};
    int timeout_ms_ {1000};
    std::string last_error_;
};

}   // namespace camera

# endif
