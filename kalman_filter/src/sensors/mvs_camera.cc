# include "sensors/mvs_camera.hpp"

# include <MvCameraControl.h>
# include <opencv2/imgproc.hpp>

# include <algorithm>
# include <chrono>
# include <cstring>
# include <iomanip>
# include <limits>
# include <mutex>
# include <sstream>
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

    //开闸，开始出图
    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK)
    {
        return rollback(
            "MV_CC_StartGrabbing 失败: " + hexCode(ret));
    }

    grabbing_ = true;
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
    // 注意：这是“帧已经躺进内存”的时刻（t_frame_out），不是曝光时刻（t_exp）。
    // 真要曝光时刻，得用 device_timestamp 或 chunk，别拿它凑合。
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
    const std::size_t source_size =
        info.nFrameLenEx != 0 ? info.nFrameLenEx : info.nFrameLen;

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

    // 时间戳三件套，各说各话，别混着算：
    //   timestamp          主机单调钟的“收帧”时刻，算法目前用它
    //   device_timestamp   相机自己的硬件戳，单位和零点要现场标定
    //   sdk_host_timestamp SDK 打的主机戳，仍是收帧时刻，不是曝光时刻
    frame.timestamp = host_time;
    frame.time_domain = core::TimeDomain::HostMonotonic;
    frame.device_timestamp =
        (static_cast<uint64_t>(info.nDevTimeStampHigh) << 32) |
        static_cast<uint64_t>(info.nDevTimeStampLow);
    frame.sdk_host_timestamp = info.nHostTimeStamp;
    frame.frame_number = info.nFrameNum;
    // 不开 Chunk-Exposure 时这货多半是 0，别指望它。
    frame.exposure_time_us = info.fExposureTime;

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
