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

std::mutex g_sdk_mutex;
int g_sdk_users {};

double steadyNowSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string hexCode(int code)
{
    std::ostringstream out;
    out << "0x" << std::hex << std::uppercase << code;
    return out.str();
}

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
    if (g_sdk_users == 0)
    {
        MV_CC_Finalize();
    }
}

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

CameraInfo toCameraInfo(
    unsigned int index,
    const MV_CC_DEVICE_INFO& info)
{
    CameraInfo out;
    out.index = index;
    out.transport_layer = info.nTLayerType;

    switch (info.nTLayerType)
    {
        case MV_GIGE_DEVICE:
        case MV_GENTL_GIGE_DEVICE:
            out.model = fixedString(
                info.SpecialInfo.stGigEInfo.chModelName,
                sizeof(info.SpecialInfo.stGigEInfo.chModelName));
            out.serial = fixedString(
                info.SpecialInfo.stGigEInfo.chSerialNumber,
                sizeof(info.SpecialInfo.stGigEInfo.chSerialNumber));
            out.user_name = fixedString(
                info.SpecialInfo.stGigEInfo.chUserDefinedName,
                sizeof(info.SpecialInfo.stGigEInfo.chUserDefinedName));
            break;

        case MV_USB_DEVICE:
            out.model = fixedString(
                info.SpecialInfo.stUsb3VInfo.chModelName,
                sizeof(info.SpecialInfo.stUsb3VInfo.chModelName));
            out.serial = fixedString(
                info.SpecialInfo.stUsb3VInfo.chSerialNumber,
                sizeof(info.SpecialInfo.stUsb3VInfo.chSerialNumber));
            out.user_name = fixedString(
                info.SpecialInfo.stUsb3VInfo.chUserDefinedName,
                sizeof(info.SpecialInfo.stUsb3VInfo.chUserDefinedName));
            break;

        default:
            out.model = "unsupported transport layer";
            break;
    }

    return out;
}

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

MvsCamera::MvsCamera(const MvsCameraConfig& config)
    : config_(config)
{
}

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

    MV_CC_DEVICE_INFO_LIST devices {};
    int ret = MV_CC_EnumDevices(commonDeviceTypes(), &devices);
    if (ret != MV_OK)
    {
        return fail(
            "MV_CC_EnumDevices 失败: " + hexCode(ret),
            error);
    }

    if (config_.device_index >= devices.nDeviceNum ||
        devices.pDeviceInfo[config_.device_index] == nullptr)
    {
        std::ostringstream message;
        message << "相机索引 " << config_.device_index
                << " 无效，检测到 " << devices.nDeviceNum << " 台";
        return fail(message.str(), error);
    }

    ret = MV_CC_CreateHandle(
        &handle_, devices.pDeviceInfo[config_.device_index]);
    if (ret != MV_OK)
    {
        handle_ = nullptr;
        return fail(
            "MV_CC_CreateHandle 失败: " + hexCode(ret),
            error);
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK)
    {
        return fail(
            "MV_CC_OpenDevice 失败: " + hexCode(ret),
            error);
    }

    if (devices.pDeviceInfo[config_.device_index]->nTLayerType ==
        MV_GIGE_DEVICE)
    {
        const int packet_size = MV_CC_GetOptimalPacketSize(handle_);
        if (packet_size > 0)
        {
            MV_CC_SetIntValueEx(
                handle_, "GevSCPSPacketSize", packet_size);
        }
    }

    ret = MV_CC_SetEnumValue(handle_, "TriggerMode", 0);
    if (ret != MV_OK)
    {
        return fail(
            "设置 TriggerMode=Off 失败: " + hexCode(ret),
            error);
    }

    if (!applyConfig(config_, error)) return false;

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK)
    {
        return fail(
            "MV_CC_StartGrabbing 失败: " + hexCode(ret),
            error);
    }

    grabbing_ = true;
    info_.width = config_.width;
    info_.height = config_.height;
    info_.frame_rate = config_.frame_rate;
    return true;
}

core::FrameReadStatus MvsCamera::read(
    CameraFrame& frame,
    std::string* error)
{
    frame = CameraFrame {};
    if (handle_ == nullptr || !grabbing_)
    {
        fail("相机尚未打开或尚未开始取流", error);
        return core::FrameReadStatus::Error;
    }

    MV_FRAME_OUT raw {};
    int ret = MV_CC_GetImageBuffer(
        handle_,
        &raw,
        static_cast<unsigned int>(
            std::max(1, config_.timeout_ms)));
    const double host_time = steadyNowSeconds();

    if (ret != MV_OK)
    {
        if (ret == MV_E_NODATA || ret == MV_E_TIMEOUT)
        {
            return core::FrameReadStatus::Timeout;
        }

        fail("MV_CC_GetImageBuffer 失败: " + hexCode(ret), error);
        return core::FrameReadStatus::Error;
    }

    bool converted = false;
    const auto& info = raw.stFrameInfo;
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
        const std::size_t output_size =
            static_cast<std::size_t>(width) * height * 3 + 2048;
        if (output_size > std::numeric_limits<unsigned int>::max())
        {
            MV_CC_FreeImageBuffer(handle_, &raw);
            fail("BGR 转换缓冲区过大", error);
            return core::FrameReadStatus::Error;
        }

        std::vector<unsigned char> bgr(output_size);
        MV_CC_PIXEL_CONVERT_PARAM_EX param {};
        param.nWidth = width;
        param.nHeight = height;
        param.enSrcPixelType = info.enPixelType;
        param.pSrcData = raw.pBufAddr;
        param.nSrcDataLen = static_cast<unsigned int>(source_size);
        param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        param.pDstBuffer = bgr.data();
        param.nDstBufferSize = static_cast<unsigned int>(bgr.size());

        ret = MV_CC_ConvertPixelTypeEx(handle_, &param);
        if (ret == MV_OK &&
            param.nDstLen >=
                static_cast<unsigned int>(width) * height * 3)
        {
            frame.bgr = cv::Mat(
                static_cast<int>(height),
                static_cast<int>(width),
                CV_8UC3,
                bgr.data()).clone();
            converted = true;
        }
    }

    frame.timestamp = host_time;
    frame.time_domain = core::TimeDomain::HostMonotonic;
    frame.device_timestamp =
        (static_cast<uint64_t>(info.nDevTimeStampHigh) << 32) |
        static_cast<uint64_t>(info.nDevTimeStampLow);
    frame.sdk_host_timestamp = info.nHostTimeStamp;
    frame.frame_number = info.nFrameNum;
    frame.exposure_time_us = info.fExposureTime;

    MV_CC_FreeImageBuffer(handle_, &raw);

    if (!converted)
    {
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
    if (handle_ != nullptr)
    {
        if (grabbing_)
        {
            MV_CC_StopGrabbing(handle_);
            grabbing_ = false;
        }

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
        if (!setRequestedEnum(
                handle_, "AcquisitionFrameRateEnable", 1, error) ||
            !setRequestedFloat(
                handle_, "AcquisitionFrameRate",
                config.frame_rate, error))
        {
            return false;
        }
    }

    if (config.exposure_time_us > 0.0F)
    {
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
