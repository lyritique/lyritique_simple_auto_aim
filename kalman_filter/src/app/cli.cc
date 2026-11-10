# include "app/cli.hpp"

# include <charconv>
# include <ostream>
# include <string_view>
# include <vector>

namespace app
{
namespace
{

bool parseFloat(std::string_view text, float& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

bool parseDouble(std::string_view text, double& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

bool parseInt(std::string_view text, int& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

bool parseUInt(std::string_view text, unsigned int& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

bool applyLegacyArguments(
    const std::vector<std::string>& args,
    core::AppConfig& config)
{
    if (args.empty() || args[0].starts_with("--"))
    {
        return true;
    }

    config.mode = core::RunMode::Replay;
    config.video_path = args[0];
    if (args.size() >= 2) config.imu_path = args[1];
    if (args.size() >= 4) config.output_directory = args[3];
    return true;
}

}   // namespace

CliResult parseCommandLine(int argc, char** argv)
{
    CliResult result;
    result.config.mode = core::RunMode::Replay;

    const std::vector<std::string> args(argv + 1, argv + argc);
    if (!applyLegacyArguments(args, result.config))
    {
        result.error = "无法解析位置参数";
        return result;
    }

    for (std::size_t i = 0; i < args.size(); ++i)
    {
        const std::string& arg = args[i];
        const bool has_value = i + 1 < args.size();

        auto requireValue = [&]() -> const std::string*
        {
            if (!has_value)
            {
                result.error = arg + " 缺少参数";
                return nullptr;
            }
            return &args[++i];
        };

        if (!arg.starts_with("--"))
        {
            continue;
        }

        if (arg == "--help" || arg == "-h")
        {
            result.help = true;
            continue;
        }
        if (arg == "--list-cameras")
        {
            result.list_cameras = true;
            continue;
        }
        if (arg == "--replay")
        {
            result.config.mode = core::RunMode::Replay;
            continue;
        }
        if (arg == "--live")
        {
            result.config.mode = core::RunMode::Live;
            continue;
        }
        if (arg == "--video")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.video_path = *value;
            continue;
        }
        if (arg == "--imu")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.imu_path = *value;
            continue;
        }
        if (arg == "--output")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.output_directory = *value;
            continue;
        }
        if (arg == "--serial")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.serial.device = *value;
            continue;
        }
        if (arg == "--baud")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseInt(*value, result.config.serial.baud))
            {
                result.error = "无效的波特率: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--comm-latency")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseDouble(
                    *value,
                    result.config.serial.communication_latency))
            {
                result.error = "无效的通信延迟: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-index")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseUInt(*value, result.config.camera.device_index))
            {
                result.error = "无效的相机索引: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-width")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseInt(*value, result.config.camera.width))
            {
                result.error = "无效的相机宽度: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-height")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseInt(*value, result.config.camera.height))
            {
                result.error = "无效的相机高度: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-fps")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseFloat(*value, result.config.camera.frame_rate))
            {
                result.error = "无效的相机帧率: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--exposure-us")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseFloat(
                    *value,
                    result.config.camera.exposure_time_us))
            {
                result.error = "无效的曝光时间: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--gain")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseFloat(*value, result.config.camera.gain))
            {
                result.error = "无效的增益: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-time-source")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (*value == "host")
            {
                result.config.camera.use_device_timestamp = false;
            }
            else if (*value == "device")
            {
                result.config.camera.use_device_timestamp = true;
            }
            else
            {
                result.error = "时间源只能是 host 或 device: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-time-offset-us")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseDouble(
                    *value,
                    result.config.camera.timestamp_offset_us))
            {
                result.error = "无效的时间偏移: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--camera-exposure-anchor")
        {
            const auto* value = requireValue();
            if (!value) break;
            float anchor = 0.0F;
            if (!parseFloat(*value, anchor) || anchor < 0.0F ||
                anchor > 1.0F)
            {
                result.error = "曝光锚点必须在 0~1 之间: " + *value;
                break;
            }
            result.config.camera.exposure_anchor = anchor;
            continue;
        }
        if (arg == "--no-chunk-exposure")
        {
            result.config.camera.chunk_exposure = false;
            continue;
        }
        if (arg == "--camera-timing-log")
        {
            result.config.camera.timing_log = true;
            continue;
        }
        if (arg == "--max-distance")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!parseDouble(*value, result.config.target_max_distance))
            {
                result.error = "无效的最大距离: " + *value;
                break;
            }
            continue;
        }
        if (arg == "--imu-offset-ms")
        {
            const auto* value = requireValue();
            if (!value) break;
            double milliseconds = 0.0;
            if (!parseDouble(*value, milliseconds))
            {
                result.error = "无效的 IMU 偏移: " + *value;
                break;
            }
            result.config.timing.replay_imu_offset_seconds =
                milliseconds / 1000.0;
            continue;
        }

        result.error = "未知参数: " + arg;
        break;
    }

    return result;
}

void printUsage(std::ostream& out)
{
    out <<
        "用法:\n"
        "  kalman_filter --replay [选项]\n"
        "  kalman_filter --live [选项]\n"
        "  kalman_filter --list-cameras\n\n"
        "通用选项:\n"
        "  --list-cameras        列出已连接的 MVS 相机\n"
        "  --help                显示帮助\n\n"
        "回放选项:\n"
        "  --video PATH          离线视频\n"
        "  --imu PATH            IMU 时间序列\n"
        "  --output PATH         输出目录\n"
        "  --imu-offset-ms MS    IMU 对齐偏移，默认 9 ms\n\n"
        "实时选项:\n"
        "  --camera-index N      MVS 相机索引\n"
        "  --camera-width N      设置相机宽度，0 表示不修改\n"
        "  --camera-height N     设置相机高度，0 表示不修改\n"
        "  --camera-fps F        设置相机帧率，0 表示不修改\n"
        "  --exposure-us US      设置曝光时间，0 表示不修改\n"
        "  --gain G              设置增益，负数表示不修改\n"
        "  --camera-time-source S  host 或 device，默认 device\n"
        "  --camera-time-offset-us US  帧时间常数偏移，台架标定用\n"
        "  --camera-exposure-anchor R  设备戳在曝光区间的比例，0~1，默认 0.5\n"
        "  --no-chunk-exposure   关闭 Chunk-Exposure，不读每帧曝光\n"
        "  --camera-timing-log   打印时钟校准结果和每帧时间戳\n"
        "  --serial PATH         串口设备\n"
        "  --baud N              串口波特率\n"
        "  --comm-latency S      通信延迟估计，秒\n"
        "  --max-distance M      送入 EKF 的最大目标距离\n";
}

}   // namespace app
