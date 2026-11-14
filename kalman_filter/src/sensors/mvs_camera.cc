# include "sensors/mvs_camera.hpp"

# include <MvCameraControl.h>
# include <opencv2/imgproc.hpp>

# include <algorithm>
# include <chrono>
# include <cstring>
# include <iomanip>
# include <iostream>
# include <limits>
# include <mutex>
# include <sstream>
# include <thread>
# include <vector>

namespace camera
{
namespace
{

// MVS SDK 是进程级全局货：第一个用户 Initialize，最后一个用户 Finalize。
// 用计数而不是“每台相机各初始化一次”，否则多相机和枚举会互相拆台。
std::mutex g_sdk_mutex;
int g_sdk_users {};

// 主机单调钟。只回答“帧什么时候到”，不回答“什么时候曝光”——后者得另想办法。
double steadyNowSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// MVS 错误码清一色 0x8000xxxx，十六进制比十进制好认。
std::string hexCode(int code)
{
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << code;
    return out.str();
}

// 节点读不到设备戳时，拿多少帧来估时钟。100 帧 ≈ 5 秒（20 fps），
// 窗口太短的话斜率误差会变成每秒几毫秒的漂移。
constexpr int kFrameCalibFrames = 100;

// 设备信息里的 char[] 不保证带 '\0'，按长度找零再截断。
std::string fixedString(const unsigned char* data, std::size_t size)
{
    const auto* text = reinterpret_cast<const char*>(data);
    const auto end = std::find(data, data + size, 0);
    const std::size_t length =
        static_cast<std::size_t>(end - data);
    return std::string(text, length);
}

bool acquireSdk(std::string* error)
{
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    //第一次给SDK初始化
    if (g_sdk_users == 0)
    {
        const int ret = MV_CC_Initialize();
        if (ret != MV_OK)
        {
            if (error)
            {
                *error = "MV_CC_Initialize 失败: " + hexCode(ret);
            }
            return false;
        }
    }

    ++g_sdk_users;
    return true;
}

void releaseSdk()
{
    std::lock_guard<std::mutex> lock(g_sdk_mutex);
    if (g_sdk_users <= 0) return;

    --g_sdk_users;
    //没有线程使用时，SDK释放
    if (g_sdk_users == 0)
    {
        MV_CC_Finalize();
    }
}

// 能枚举的全带上；实际能不能开，得看相机和驱动给不给面子。
unsigned int commonDeviceTypes()
{
    return MV_GIGE_DEVICE |
           MV_USB_DEVICE |
           MV_GENTL_GIGE_DEVICE |
           MV_GENTL_CAMERALINK_DEVICE |
           MV_GENTL_CXP_DEVICE |
           MV_GENTL_XOF_DEVICE |
           MV_GENTL_XOC_DEVICE;
}

// 传输层常量是位掩码，直接打印只会得到一个数字。
const char* transportName(unsigned int layer)
{
    switch (layer)
    {
        case MV_GIGE_DEVICE: return "GigE";
        case MV_USB_DEVICE: return "USB3";
        case MV_GENTL_GIGE_DEVICE: return "GenTL-GigE";
        case MV_GENTL_CAMERALINK_DEVICE: return "CameraLink";
        case MV_GENTL_CXP_DEVICE: return "CoaXPress";
        case MV_GENTL_XOF_DEVICE: return "XoF";
        case MV_GENTL_XOC_DEVICE: return "XoC";
        default: return "未知";
    }
}

// 各传输层的信息结构字段名一样、只是类型不同，拿模板抄一遍就够了。
template <typename DeviceInfo>
void fillDeviceInfo(const DeviceInfo& device, CameraInfo& out)
{
    out.model = fixedString(
        device.chModelName,
        sizeof(device.chModelName));
    out.serial = fixedString(
        device.chSerialNumber,
        sizeof(device.chSerialNumber));
    out.user_name = fixedString(
        device.chUserDefinedName,
        sizeof(device.chUserDefinedName));
}

// 从像素类型里抠出“每像素占多少 bit”，8/12/16/24 都能算。
unsigned int pixelBits(MvGvspPixelType type)
{
    return (static_cast<unsigned int>(type) &
            MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_MASK) >>
           MV_GVSP_PIX_EFFECTIVE_PIXEL_SIZE_SHIFT;
}

CameraInfo toCameraInfo(
    unsigned int index,
    const MV_CC_DEVICE_INFO& info)
{
    CameraInfo out;
    out.index = index;
    out.transport_layer = info.nTLayerType;
    out.transport = transportName(info.nTLayerType);

    switch (info.nTLayerType)
    {
        case MV_GIGE_DEVICE:
        case MV_GENTL_GIGE_DEVICE:
            fillDeviceInfo(info.SpecialInfo.stGigEInfo, out);
            break;

        case MV_USB_DEVICE:
            fillDeviceInfo(info.SpecialInfo.stUsb3VInfo, out);
            break;

        case MV_GENTL_CAMERALINK_DEVICE:
            fillDeviceInfo(info.SpecialInfo.stCMLInfo, out);
            break;

        case MV_GENTL_CXP_DEVICE:
            fillDeviceInfo(info.SpecialInfo.stCXPInfo, out);
            break;

        case MV_GENTL_XOF_DEVICE:
            fillDeviceInfo(info.SpecialInfo.stXoFInfo, out);
            break;

        case MV_GENTL_XOC_DEVICE:
            fillDeviceInfo(info.SpecialInfo.stXoCInfo, out);
            break;

        default:
            out.model = "未知传输层";
            break;
    }

    return out;
}

// 下面四个小函数只做一件事：设置节点，失败时把错误码翻译成人话。

bool setRequestedInt(
    void* handle,
    const char* key,
    int value,
    std::string* error)
{
    const int ret = MV_CC_SetIntValueEx(handle, key, value);
    if (ret == MV_OK) return true;
    if (error)
    {
        *error = std::string("设置 ") + key + " 失败: " + hexCode(ret);
    }
    return false;
}

bool setRequestedFloat(
    void* handle,
    const char* key,
    float value,
    std::string* error)
{
    const int ret = MV_CC_SetFloatValue(handle, key, value);
    if (ret == MV_OK) return true;
    if (error)
    {
        *error = std::string("设置 ") + key + " 失败: " + hexCode(ret);
    }
    return false;
}

bool setRequestedBool(
    void* handle,
    const char* key,
    bool value,
    std::string* error)
{
    const int ret = MV_CC_SetBoolValue(handle, key, value);
    if (ret == MV_OK) return true;
    if (error)
    {
        *error = std::string("设置 ") + key + " 失败: " + hexCode(ret);
    }
    return false;
}

bool setRequestedEnum(
    void* handle,
    const char* key,
    unsigned int value,
    std::string* error)
{
    const int ret = MV_CC_SetEnumValue(handle, key, value);
    if (ret == MV_OK) return true;
    if (error)
    {
        *error = std::string("设置 ") + key + " 失败: " + hexCode(ret);
    }
    return false;
}


}   // namespace

//构造
MvsCamera::MvsCamera(const MvsCameraConfig& config)
    : config_(config)
{
}

//析构
MvsCamera::~MvsCamera()
{
    close();
}



std::vector<CameraInfo> MvsCamera::enumerate(std::string* error)
{
    std::vector<CameraInfo> result;
    if (!acquireSdk(error)) return result;

    MV_CC_DEVICE_INFO_LIST devices {};
    const int ret = MV_CC_EnumDevices(commonDeviceTypes(), &devices);
    if (ret != MV_OK)
    {
        if (error)
        {
            *error = "MV_CC_EnumDevices 失败: " + hexCode(ret);
        }
        releaseSdk();
        return result;
    }

    // devices 里的指针归 SDK 所有，Finalize 后就作废；
    // 所以先拷进 CameraInfo，再 releaseSdk()。
    result.reserve(devices.nDeviceNum);
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i)
    {
        if (devices.pDeviceInfo[i] != nullptr)
        {
            result.push_back(toCameraInfo(i, *devices.pDeviceInfo[i]));
        }
    }

    releaseSdk();
    return result;
}


bool MvsCamera::open(std::string* error)
{
    close();
    last_error_.clear();

    if (!acquireSdk(error)) return false;
    sdk_acquired_ = true;

    // 任何一步失败都原地收拾干净：关句柄、退 SDK，
    const auto rollback = [this, error](const std::string& reason)
    {
        close();
        return fail(reason, error);
    };

    // MV_CC_EnumDevices
    MV_CC_DEVICE_INFO_LIST devices {};
    int ret = MV_CC_EnumDevices(commonDeviceTypes(), &devices);
    if (ret != MV_OK)
    {
        return rollback(
            "MV_CC_EnumDevices 失败: " + hexCode(ret));
    }

    if (config_.device_index >= devices.nDeviceNum ||
        devices.pDeviceInfo[config_.device_index] == nullptr)
    {
        std::ostringstream message;
        message << "相机索引 " << config_.device_index
                << " 无效，检测到 " << devices.nDeviceNum << " 台";
        return rollback(message.str());
    }

    // MV_CC_CreateHandle
    ret = MV_CC_CreateHandle(
        &handle_, devices.pDeviceInfo[config_.device_index]);
    if (ret != MV_OK)
    {
        handle_ = nullptr;
        return rollback(
            "MV_CC_CreateHandle 失败: " + hexCode(ret));
    }

    // MV_CC_OpenDevice
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK)
    {
        return rollback(
            "MV_CC_OpenDevice 失败: " + hexCode(ret));
    }

    //配置如何拍摄
    if (devices.pDeviceInfo[config_.device_index]->nTLayerType ==
        MV_GIGE_DEVICE)
    {
        // 包太小容易掉帧，包太大网卡顶不住, SDK 自己算。
        const int packet_size = MV_CC_GetOptimalPacketSize(handle_);
        if (packet_size > 0)
        {
            MV_CC_SetIntValueEx(
                handle_, "GevSCPSPacketSize", packet_size);
        }
    }
    // 自由跑，不等外触发；硬件同步那套留给以后真要用的时候。
    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (ret != MV_OK)
    {
        return rollback(
            "设置 TriggerMode=Off 失败: " + hexCode(ret));
    }

    if (!applyConfig(config_, error))
    {
        const std::string reason =
            error != nullptr && !error->empty()
                ? *error
                : "相机参数配置失败";
        return rollback(reason);
    }

    if (config_.chunk_exposure)
    {
        std::string warning;
        if (!enableChunkExposure(&warning))
        {
            std::clog << "警告: " << warning
                      << "，每帧曝光时间不可用\n";
        }
    }
    else
    {
        // 上次进程可能把 ChunkMode 留成开着的，先确认关掉，否则帧长会变。
        MV_CC_SetBoolValue(handle_, "ChunkModeActive", false);
    }

    //开闸，开始出图
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK)
    {
        return rollback(
            "MV_CC_StartGrabbing 失败: " + hexCode(ret));
    }

    grabbing_ = true;

    // 取流开始后再标定，万一 StartGrabbing 让设备戳归零也能兜住。
    if (config_.use_device_timestamp)
    {
        std::string clock_error;
        if (!calibrateDeviceClock(&clock_error))
        {
            std::clog << "警告: 设备时钟校准失败（" << clock_error
                      << "），退回收帧时刻\n";
        }
        else if (!clock_ready_)
        {
            std::clog << "提示: DeviceTimestamp 节点不可用，改用 ";
            if (calib_seconds_ > 0.0)
            {
                std::clog << calib_seconds_ << " 秒";
            }
            else
            {
                std::clog << calib_frames_left_ << " 帧";
            }
            std::clog << " 在线校准\n";
        }
    }

    if (config_.timing_log)
    {
        std::clog << "[相机时钟] mode="
                  << (clock_ready_ ? "device"
                                   : (calib_frames_left_ > 0 ? "calibrating"
                                                             : "host"))
                  << " ns/tick=" << clock_scale_ * 1e9
                  << " offset=" << clock_offset_
                  << " spread=" << clock_spread_ms_ << " ms\n";
    }

    // 先记配置值顶一下；真实宽高要等第一帧到了才准。
    info_.width = config_.width;
    info_.height = config_.height;
    info_.frame_rate = config_.frame_rate;
    return true;
}

core::FrameReadStatus MvsCamera::read(
    CameraFrame& frame,
    std::string* error)
{
    // 每帧从干净状态开始，上一帧的 Mat 该释放就释放，别串台。
    frame = CameraFrame {};
    if (handle_ == nullptr || !grabbing_)
    {
        // 没开流就老实报错
        fail("相机尚未打开或尚未开始取流", error);
        return core::FrameReadStatus::Error;
    }

    MV_FRAME_OUT raw {};
    int ret = MV_CC_GetImageBuffer(
        handle_,
        &raw,
        static_cast<unsigned int>(
            std::max(1, config_.timeout_ms)));
    // 收帧时刻只当兜底：设备时钟校准成功后，会用相机自己的戳。
    const double host_time = steadyNowSeconds();

    if (ret != MV_OK)
    {
        // 超时是正常剧情：相机这轮没新帧，不算故障。
        if (ret == MV_E_NODATA || ret == MV_E_TIMEOUT)
        {
            return core::FrameReadStatus::Timeout;
        }

        fail("MV_CC_GetImageBuffer 失败: " + hexCode(ret), error);
        return core::FrameReadStatus::Error;
    }

    //打包，写的有些繁复

    bool converted = false;
    const auto& info = raw.stFrameInfo;
    // 新 SDK 用扩展字段，老 SDK 只有 16 位字段，两套都认。
    const unsigned int width =
        info.nExtendWidth != 0 ? info.nExtendWidth : info.nWidth;
    const unsigned int height =
        info.nExtendHeight != 0 ? info.nExtendHeight : info.nHeight;
    std::size_t source_size =
        info.nFrameLenEx != 0 ? info.nFrameLenEx : info.nFrameLen;
    if (chunk_active_ && chunk_bits_per_pixel_ > 0)
    {
        // 开 chunk 后 nFrameLenEx 尾巴上挂着元数据，转换只能喂图像本身。
        const std::size_t image_size =
            (static_cast<std::size_t>(width) * height *
                 chunk_bits_per_pixel_ + 7) / 8;
        if (image_size > 0 && image_size <= source_size)
        {
            source_size = image_size;
        }
    }

    if (width == 0 || height == 0 || raw.pBufAddr == nullptr)
    {
        MV_CC_FreeImageBuffer(handle_, &raw);
        fail("相机返回了空图像", error);
        return core::FrameReadStatus::Error;
    }

    if (info.enPixelType == PixelType_Gvsp_BGR8_Packed)
    {
        // 原厂 buffer 还回去就没了，所以这里必须 clone 一份自己的。
        frame.bgr = cv::Mat(
            static_cast<int>(height),
            static_cast<int>(width),
            CV_8UC3,
            raw.pBufAddr).clone();
        converted = true;
    }
    else if (info.enPixelType == PixelType_Gvsp_RGB8_Packed)
    {
        const cv::Mat rgb(
            static_cast<int>(height),
            static_cast<int>(width),
            CV_8UC3,
            raw.pBufAddr);
        cv::cvtColor(rgb, frame.bgr, cv::COLOR_RGB2BGR);
        converted = true;
    }
    else if (info.enPixelType == PixelType_Gvsp_Mono8)
    {
        const cv::Mat gray(
            static_cast<int>(height),
            static_cast<int>(width),
            CV_8UC1,
            raw.pBufAddr);
        cv::cvtColor(gray, frame.bgr, cv::COLOR_GRAY2BGR);
        converted = true;
    }
    else
    {
        const std::size_t required_size =
            static_cast<std::size_t>(width) * height * 3;
        if (required_size > std::numeric_limits<unsigned int>::max())
        {
            MV_CC_FreeImageBuffer(handle_, &raw);
            fail("BGR 转换缓冲区过大", error);
            return core::FrameReadStatus::Error;
        }
        const auto required_bytes =
            static_cast<unsigned int>(required_size);

        // Bayer 之类交给 SDK 转换。目标直接指向 frame.bgr，
        // 省掉临时 buffer 和一次整图 clone（1440x1080 每帧少搬 4.6 MB）。
        frame.bgr.create(
            static_cast<int>(height),
            static_cast<int>(width),
            CV_8UC3);

        MV_CC_PIXEL_CONVERT_PARAM_EX param {};
        param.nWidth = width;
        param.nHeight = height;
        param.enSrcPixelType = info.enPixelType;
        param.pSrcData = raw.pBufAddr;
        param.nSrcDataLen = static_cast<unsigned int>(source_size);
        param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        param.pDstBuffer = frame.bgr.data;
        param.nDstBufferSize = required_bytes;

        ret = MV_CC_ConvertPixelTypeEx(handle_, &param);
        if (ret == MV_OK && param.nDstLen >= required_bytes)
        {
            converted = true;
        }
        else
        {
            frame.bgr.release();
        }
    }

    const uint64_t device_ticks =
        (static_cast<uint64_t>(info.nDevTimeStampHigh) << 32) |
        static_cast<uint64_t>(info.nDevTimeStampLow);

    // Chunk-Exposure 没开成时，退回配置里的固定曝光，至少能做中点修正。
    const float exposure_us =
        info.fExposureTime > 0.0F
            ? info.fExposureTime
            : config_.exposure_time_us;

    // 节点校准不了（戳恒为 0 或只在出帧时跳）就用真实帧补：
    // 斜率（每秒多少 tick）必须从时间跨度里拟合出来 —— 相机报的 Increment 只是标称值，
    // 实际可能差百分之一，按标称值算会以每秒几毫秒的速度累积漂移。
    // 按时间收样本是长时间标定用的：窗口越长，收帧抖动被摊得越薄，斜率越准。
    const bool time_based = calib_seconds_ > 0.0;
    if (!clock_ready_ && (time_based || calib_frames_left_ > 0) &&
        config_.use_device_timestamp && device_ticks != 0)
    {
        // 中途卡顿（TensorRT 首次建引擎、USB 抽风）会让这一段的斜率完全失真，
        // 只要相邻样本间隔异常就丢掉整段重新攒。
        constexpr double kMaxSampleGap = 0.5;
        if (!calib_hosts_.empty() &&
            host_time - calib_hosts_.back() > kMaxSampleGap)
        {
            calib_ticks_.clear();
            calib_hosts_.clear();
            calib_frames_left_ = kFrameCalibFrames;
        }

        calib_ticks_.push_back(device_ticks);
        calib_hosts_.push_back(host_time);

        bool finished = false;
        if (time_based)
        {
            finished = !calib_hosts_.empty() &&
                (host_time - calib_hosts_.front() >= calib_seconds_);
        }
        else
        {
            finished = (--calib_frames_left_ == 0);
        }

        if (finished)
        {
            const std::size_t n = calib_ticks_.size();
            double mean_t = 0.0;
            double mean_h = 0.0;
            for (std::size_t i = 0; i < n; ++i)
            {
                mean_t += static_cast<double>(calib_ticks_[i]);
                mean_h += calib_hosts_[i];
            }
            mean_t /= static_cast<double>(n);
            mean_h /= static_cast<double>(n);

            double cov = 0.0;
            double var = 0.0;
            for (std::size_t i = 0; i < n; ++i)
            {
                const double dt =
                    static_cast<double>(calib_ticks_[i]) - mean_t;
                cov += dt * (calib_hosts_[i] - mean_h);
                var += dt * dt;
            }

            // 斜率 = 每 tick 多少秒。已经给定 tick_ns 就不再拟合，避免被窗口噪声带偏。
            // 节点已经给出可信斜率时不再用帧数据覆盖它 —— 收帧抖动只会把斜率带偏。
            // 但仍然算一个实测值打出来，方便和节点值对账。
            const double fitted = var > 0.0 ? cov / var : 0.0;
            const bool fitted_ok = fitted >= 5e-9 && fitted <= 2e-8;
            if (config_.tick_ns <= 0.0 && !scale_from_node_ && fitted_ok)
            {
                clock_scale_ = fitted;
            }

            // 截距取残差的最小值：传输延迟只会让「收帧时刻 - 设备时刻」变大，
            // 最小值最接近真实的时钟偏差。
            double best = 0.0;
            bool has_best = false;
            for (std::size_t i = 0; i < n; ++i)
            {
                const double d =
                    calib_hosts_[i] -
                    static_cast<double>(calib_ticks_[i]) * clock_scale_;
                if (!has_best || d < best)
                {
                    best = d;
                    has_best = true;
                }
            }

            clock_offset_ = best;
            clock_spread_ms_ = 0.0;
            clock_ready_ = true;
            std::clog << "[相机时钟] 校准完成 ns/tick="
                      << clock_scale_ * 1e9
                      << (scale_from_node_ ? "（节点值）" : "（拟合值）")
                      << " 实测=" << (fitted_ok ? fitted * 1e9 : 0.0)
                      << " 样本=" << n << '\n';
            // 校准期间 last_frame_timestamp_ 记的是收帧时刻，换到设备时刻会"倒退"，
            // 不清掉就会被单调性护栏一直钳成 host_time。
            last_frame_timestamp_ = 0.0;
            calib_ticks_.clear();
            calib_hosts_.clear();
        }
    }

    // 滞后一路涨说明两件事之一：相机晶振和主机不一致（比例算错了），
    // 或者取帧跟不上、SDK 内部积压（那是真延迟）。两者都会让预测偏，
    // 但修法完全不同，所以先报警让人别当成正常现象。
    if (clock_ready_ && device_ticks != 0 && last_host_time_ > 0.0)
    {
        const double lag = host_time - (deviceTicksToHost(device_ticks));
        if (lag > 0.1 && lag - last_warned_lag_ > 0.05)
        {
            last_warned_lag_ = lag;
            std::clog << "警告: 帧时间戳滞后 " << lag * 1000.0
                      << " ms 且持续增大，检查相机取帧是否积压\n";
        }
    }
    if (clock_ready_) last_host_time_ = host_time;

    double stamp = host_time;
    if (config_.use_device_timestamp && clock_ready_ && device_ticks != 0)
    {
        stamp = deviceTicksToHost(device_ticks);

        // anchor=0 表示戳在曝光开始，0.5 在中点，1 在结束。
        if (exposure_us > 0.0F)
        {
            stamp +=
                (0.5 - static_cast<double>(config_.exposure_anchor)) *
                static_cast<double>(exposure_us) * 1e-6;
        }

        // 映射时刻不可能晚于收帧时刻，也不该掉到很久以前。
        // 超出 1 秒说明设备戳复位或校准坏了，直接回退。
        if (stamp > host_time || stamp < host_time - 1.0)
        {
            stamp = host_time;
        }

        // 设备戳回绕/复位时保底，不能喂给 EKF 倒退或相等的时间。
        if (last_frame_timestamp_ > 0.0 &&
            stamp <= last_frame_timestamp_)
        {
            stamp = std::max(
                host_time, last_frame_timestamp_ + 1e-6);
        }
    }
    last_frame_timestamp_ = stamp;

    if (config_.timing_log)
    {
        std::clog << "[帧] n=" << info.nFrameNum
                  << " dev=" << device_ticks
                  << " mapped=" << stamp
                  << " recv=" << host_time
                  << " lag_us=" << (host_time - stamp) * 1e6
                  << " exp_us=" << exposure_us << '\n';
    }

    // 时间戳三件套，各说各话，别混着算：
    //   timestamp          映射到主机单调钟的相机事件时刻
    //   device_timestamp   相机自己的原始硬件戳，留着排查
    //   sdk_host_timestamp SDK 打的主机戳，仍是收帧时刻
    frame.timestamp = stamp;
    frame.time_domain = core::TimeDomain::HostMonotonic;
    frame.device_timestamp = device_ticks;
    frame.sdk_host_timestamp = info.nHostTimeStamp;
    frame.frame_number = info.nFrameNum;
    frame.exposure_time_us = exposure_us;

    MV_CC_FreeImageBuffer(handle_, &raw);

    if (!converted)
    {
        // 走到这儿说明相机吐了个不认识的像素格式。
        std::ostringstream message;
        message << "不支持的像素格式: "
                << static_cast<int>(info.enPixelType)
                << "，BGR 转换失败 " << hexCode(ret);
        fail(message.str(), error);
        return core::FrameReadStatus::Error;
    }

    info_.width = static_cast<int>(width);
    info_.height = static_cast<int>(height);
    return core::FrameReadStatus::Ok;
}

void MvsCamera::close()
{
    // 析构、open() 开头、失败回滚都会调它，所以必须幂等，重复调用不能出事。
    if (handle_ != nullptr)
    {
        if (grabbing_)
        {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }

        // 顺序不能反：先停流、再关设备、最后销毁句柄。SDK 对倒着来很敏感。
        MV_CC_CloseDevice(handle_);
        MV_CC_DestroyHandle(handle_);
        handle_ = nullptr;
    }

    if (sdk_acquired_)
    {
        releaseSdk();
        sdk_acquired_ = false;
    }

    chunk_active_ = false;
    chunk_bits_per_pixel_ = 0;
    clock_ready_ = false;
    clock_offset_ = 0.0;
    clock_spread_ms_ = 0.0;
    calib_frames_left_ = 0;
    calib_ticks_.clear();
    calib_hosts_.clear();
    calib_seconds_ = 0.0;
    last_frame_timestamp_ = 0.0;
}

bool MvsCamera::readDeviceTimestamp(uint64_t& ticks, std::string* error) const
{
    MVCC_INTVALUE_EX value {};
    const int ret = MV_CC_GetIntValueEx(
        handle_, "DeviceTimestamp", &value);
    if (ret != MV_OK)
    {
        if (error)
        {
            *error = "读 DeviceTimestamp 失败: " + hexCode(ret);
        }
        return false;
    }

    ticks = static_cast<uint64_t>(value.nCurValue);
    return true;
}

bool MvsCamera::calibrateDeviceClock(std::string* error)
{
    clock_ready_ = false;
    clock_offset_ = 0.0;
    clock_spread_ms_ = 0.0;
    calib_frames_left_ = 0;
    calib_ticks_.clear();
    calib_hosts_.clear();
    calib_seconds_ = config_.calib_seconds;

    // 已经用长基线标定过就直接用给定值，不再让窗口噪声左右斜率。
    if (config_.tick_ns > 0.0)
    {
        clock_scale_ = config_.tick_ns * 1e-9;
    }

    // 没给 tick_ns 时才去问节点；给了就以配置为准。
    scale_from_node_ = config_.tick_ns > 0.0;
    if (config_.tick_ns <= 0.0)
    {
        clock_scale_ = 1e-9;
        MVCC_INTVALUE_EX increment {};
        if (MV_CC_GetIntValueEx(
                handle_, "DeviceTimestampIncrement", &increment) == MV_OK &&
            increment.nCurValue > 0)
        {
            const double raw = static_cast<double>(increment.nCurValue);
            clock_scale_ = raw >= 1e6 ? 1.0 / raw : raw * 1e-9;
            scale_from_node_ = true;
        }
    }

    // 只在读节点的瞬间配对，不掺入图像传输延迟，否则等于没换。
    // 有些型号的 DeviceTimestamp 是按帧更新的，采样窗口必须跨过几个帧周期，
    // 否则连续读到同一个值会被误判成"没走动"。
    constexpr int kSamples = 24;
    constexpr double kMaxWaitSeconds = 1.0;
    std::vector<double> offsets;
    offsets.reserve(kSamples);
    uint64_t first_ticks = 0;
    uint64_t last_ticks = 0;
    int taken = 0;
    int changes = 0;

    const double deadline = steadyNowSeconds() + kMaxWaitSeconds;
    for (;;)
    {
        // 够样本且值已经走动就收工；等满一秒还没动就认输。
        if (taken >= kSamples && last_ticks > first_ticks) break;
        if (steadyNowSeconds() > deadline) break;

        const double before = steadyNowSeconds();
        uint64_t ticks = 0;
        std::string read_error;
        if (!readDeviceTimestamp(ticks, &read_error))
        {
            if (error) *error = read_error;
            return false;
        }
        const double after = steadyNowSeconds();

        if (taken == 0) first_ticks = ticks;
        else if (ticks != last_ticks) ++changes;
        last_ticks = ticks;
        ++taken;

        offsets.push_back(
            (before + after) * 0.5 -
            static_cast<double>(ticks) * clock_scale_);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (offsets.empty())
    {
        if (error) *error = "DeviceTimestamp 采样为空";
        return false;
    }

    if (last_ticks <= first_ticks)
    {
        // 节点恒为 0，或者只在出帧时跳一次，都走这里。
        // 这种型号的戳在帧信息里是好的，改用真实帧做在线校准。
        calib_frames_left_ = kFrameCalibFrames;
        return true;
    }

    std::sort(offsets.begin(), offsets.end());

    // 自由计数器每次读都在变，抖动只是噪声，取中位数最稳；
    // 按帧更新的节点（值只在帧到达时跳一次）会让 offset 呈锯齿，
    // 取最小值才对应"戳刚更新"那一刻，不会带上半个帧周期的偏差。
    const bool free_running = changes * 2 >= taken;
    if (free_running)
    {
        const std::size_t mid = offsets.size() / 2;
        clock_offset_ = offsets.size() % 2 == 0
            ? (offsets[mid - 1] + offsets[mid]) * 0.5
            : offsets[mid];
    }
    else
    {
        clock_offset_ = offsets.front();
    }
    clock_spread_ms_ = (offsets.back() - offsets.front()) * 1000.0;
    clock_ready_ = true;
    return true;
}

double MvsCamera::deviceTicksToHost(uint64_t ticks) const
{
    // 正的 timestamp_offset_us 会把帧时间往后挪，台架标定时慢慢拧。
    return static_cast<double>(ticks) * clock_scale_ + clock_offset_ +
           config_.timestamp_offset_us * 1e-6;
}

bool MvsCamera::enableChunkExposure(std::string* warning)
{
    chunk_active_ = false;
    chunk_bits_per_pixel_ = 0;

    MVCC_ENUMVALUE pixel_format {};
    if (MV_CC_GetEnumValue(
            handle_, "PixelFormat", &pixel_format) != MV_OK)
    {
        if (warning) *warning = "读不到 PixelFormat";
        return false;
    }

    const unsigned int bits = pixelBits(
        static_cast<MvGvspPixelType>(pixel_format.nCurValue));
    if (bits == 0)
    {
        if (warning) *warning = "像素格式未知";
        return false;
    }

    if (MV_CC_SetBoolValue(handle_, "ChunkModeActive", true) != MV_OK)
    {
        if (warning) *warning = "相机不支持 ChunkModeActive";
        return false;
    }

    int ret = MV_CC_SetEnumValueByString(
        handle_, "ChunkSelector", "Exposure");
    if (ret == MV_OK)
    {
        ret = MV_CC_SetBoolValue(handle_, "ChunkEnable", true);
    }

    if (ret != MV_OK)
    {
        // 半开状态最坑，失败就把 ChunkMode 关回去。
        MV_CC_SetBoolValue(handle_, "ChunkModeActive", false);
        if (warning)
        {
            *warning = "Chunk-Exposure 设置失败: " + hexCode(ret);
        }
        return false;
    }

    chunk_bits_per_pixel_ = bits;
    chunk_active_ = true;
    return true;
}

bool MvsCamera::fail(
    const std::string& message,
    std::string* error)
{
    last_error_ = message;
    if (error) *error = message;
    return false;
}

bool MvsCamera::applyConfig(
    const MvsCameraConfig& config,
    std::string* error)
{
    // TODO: 先把 OffsetX/OffsetY 清零再改宽高；
    // 相机若带着旧 ROI，直接改分辨率很可能被拒。
    if (config.width > 0 &&
        !setRequestedInt(handle_, "Width", config.width, error))
    {
        return false;
    }

    if (config.height > 0 &&
        !setRequestedInt(handle_, "Height", config.height, error))
    {
        return false;
    }

    if (config.frame_rate > 0.0F)
    {
        // AcquisitionFrameRateEnable 是 Boolean 节点，不是 Enum。
        // 用 SetEnumValue 写它会直接翻脸，--camera-fps 一开就起不来。
        if (!setRequestedBool(
                handle_, "AcquisitionFrameRateEnable", true, error) ||
            !setRequestedFloat(
                handle_, "AcquisitionFrameRate",
                config.frame_rate, error))
        {
            return false;
        }
    }

    if (config.exposure_time_us > 0.0F)
    {
        // 先关自动曝光再写死，不然你设你的，它调它的。
        if (!setRequestedEnum(handle_, "ExposureAuto", 0, error) ||
            !setRequestedFloat(
                handle_, "ExposureTime",
                config.exposure_time_us, error))
        {
            return false;
        }
    }

    if (config.gain >= 0.0F)
    {
        // 同理：自动增益不关，写的 Gain 就是摆设。
        if (!setRequestedEnum(handle_, "GainAuto", 0, error) ||
            !setRequestedFloat(
                handle_, "Gain", config.gain, error))
        {
            return false;
        }
    }

    return true;
}

}   // namespace camera
