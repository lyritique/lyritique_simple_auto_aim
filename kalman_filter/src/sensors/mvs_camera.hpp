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
    unsigned int index {};          // 设备索引，对应 --camera-index
    unsigned int transport_layer {};// 原始 nTLayerType（GigE / USB3 / GENTL…）
    std::string transport;          // 可读的传输层名字，如 "USB3" / "GigE"
    std::string model;              // 型号
    std::string serial;             // 序列号
    std::string user_name;          // 用户自定义名
};

using CameraFrame = core::TimestampedFrame;

// 海康 MVS 相机封装：枚举、开流、取一帧、关流。
// 只负责把图完整拿上来；时间同步、检测、滤波这些事一概不管。
class MvsCamera final : public CameraSource
{
public:
    explicit MvsCamera(const MvsCameraConfig& config = {});
    ~MvsCamera() override;

    MvsCamera(const MvsCamera&) = delete;
    MvsCamera& operator=(const MvsCamera&) = delete;

    // 静态枚举，不碰任何相机句柄；留着给 CLI 列设备用。
    static std::vector<CameraInfo> enumerate(std::string* error = nullptr);

    // 打开并开始取流。失败后对象回到“未打开”状态，可以直接重试。
    bool open(std::string* error = nullptr) override;

    // 阻塞读一帧。没新帧返回 Timeout，不是故障，别拿它当退出条件。
    core::FrameReadStatus read(
        CameraFrame& frame,
        std::string* error = nullptr) override;
    void close() override;

    // 句柄在手 + 正在取流，才算真的打开；open() 失败后不会谎报。
    bool isOpen() const override { return handle_ != nullptr && grabbing_; }
    SourceInfo info() const override { return info_; }

private:
    // 统一错误出口：记一份、回一份，返回值固定 false，方便 return fail(...)。
    bool fail(const std::string& message, std::string* error);

    // 约定：0 或负值表示“这项别动”，保留相机当前值。
    bool applyConfig(
        const MvsCameraConfig& config,
        std::string* error);

    // 设备时钟 -> 主机单调钟：读节点配对采样，求钟差。
    bool calibrateDeviceClock(std::string* error);
    bool readDeviceTimestamp(uint64_t& ticks, std::string* error) const;
    double deviceTicksToHost(uint64_t ticks) const;

    // 尽力打开 Chunk-Exposure；失败只警告，不影响取流。
    bool enableChunkExposure(std::string* warning);

    MvsCameraConfig config_;
    void* handle_ {};        // SDK 设备句柄
    bool grabbing_ {};       // 是否已开始取流
    bool sdk_acquired_ {};   // 是否持有 SDK 引用计数
    SourceInfo info_;
    std::string last_error_;

    // 时间戳与 chunk 状态
    bool chunk_active_ {};
    unsigned int chunk_bits_per_pixel_ {};
    bool clock_ready_ {};
    double clock_scale_ {1e-9};    // 设备 tick -> 秒
    double clock_offset_ {};       // host = device 秒 + offset
    double clock_spread_ms_ {};    // 校准样本极差，诊断用
    double last_frame_timestamp_ {};
};

}   // namespace camera

# endif
