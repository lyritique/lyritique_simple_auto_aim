# include "app/cli.hpp"

# include <opencv2/core.hpp>

# include <charconv>
# include <ostream>
# include <string_view>
# include <vector>

namespace app
{
namespace
{

// 作用：把一段文本解析成 float，解析不了就说不。实现：from_chars 全量匹配，错误码干净且指针必须吃到末尾。
bool parseFloat(std::string_view text, float& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

// 作用：文本转 double，用法同上。实现：from_chars 全量匹配，顺带挡住 "12abc" 这种半截成功。
bool parseDouble(std::string_view text, double& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

// 作用：文本转 int，给波特率、宽高这类整数参数用。实现：from_chars 加完整消费检查。
bool parseInt(std::string_view text, int& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

// 作用：文本转 unsigned，专门伺候相机索引，负数直接没门。实现：from_chars 的 unsigned 重载加全量检查。
bool parseUInt(std::string_view text, unsigned int& out)
{
    const auto result = std::from_chars(
        text.data(), text.data() + text.size(), out);
    return result.ec == std::errc {} &&
           result.ptr == text.data() + text.size();
}

// 作用：从 OpenCV 标定文件里读内参，填进配置。实现：FileStorage 读 K、D 和图像尺寸，缺项或形状不对就报错。
bool loadIntrinsicsFile(
    const std::string& path,
    core::IntrinsicsConfig& intrinsics,
    std::string& error)
{
    // 作用：打开标定 YAML，后续按节点名取数据。实现：FileStorage 只读模式，开不了直接报错。
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        error = "打不开标定文件: " + path;
        return false;
    }

    // 作用：取出相机矩阵和畸变系数，这两个是硬性要求。实现：按节点名 >> 到 Mat。
    cv::Mat camera_matrix;
    cv::Mat distortion;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion;

    // 作用：确认相机矩阵确实是 3x3，否则后面 at() 取值就危险了。实现：行列数检查。
    if (camera_matrix.rows != 3 || camera_matrix.cols != 3)
    {
        error = "标定文件缺少 3x3 camera_matrix: " + path;
        return false;
    }
    // 作用：畸变系数可以为 4/5/8 个，但不能一个都没有。实现：empty 检查。
    if (distortion.empty())
    {
        error = "标定文件缺少 distortion_coefficients: " + path;
        return false;
    }

    // 作用：统一成 double，别让 float 标定文件把精度截了。实现：convertTo CV_64F。
    camera_matrix.convertTo(camera_matrix, CV_64F);
    intrinsics.fx = camera_matrix.at<double>(0, 0);
    intrinsics.fy = camera_matrix.at<double>(1, 1);
    intrinsics.cx = camera_matrix.at<double>(0, 2);
    intrinsics.cy = camera_matrix.at<double>(1, 2);

    // 作用：把畸变系数拍成一维后逐项搬进配置。实现：先 reshape 成 1 行，再拷贝指针区间。
    distortion = distortion.reshape(1, 1);
    distortion.convertTo(distortion, CV_64F);
    intrinsics.dist.assign(
        distortion.ptr<double>(0),
        distortion.ptr<double>(0) + distortion.total());

    // 作用：图像尺寸用来后面校验帧分辨率，文件里没写就保留默认值。实现：读两个 int，都为正才覆盖。
    int width = 0;
    int height = 0;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    if (width > 0 && height > 0)
    {
        intrinsics.width = width;
        intrinsics.height = height;
    }

    // 作用：焦距是内参的命根子，非正数直接判文件不可用。实现：fx/fy 正数检查。
    if (!(intrinsics.fx > 0.0) || !(intrinsics.fy > 0.0))
    {
        error = "标定文件焦距无效: " + path;
        return false;
    }

    intrinsics.calibration_path = path;
    return true;
}

// 作用：兼容老式“直接甩三个路径”的用法，别让老脚本一夜失效。实现：首个参数不是 -- 开头就按视频、IMU、输出目录的位置依次塞。
bool applyLegacyArguments(
    const std::vector<std::string>& args,
    core::AppConfig& config)
{
    // 作用：没参数或全是开关，说明不是老式用法，直接放行。实现：空或首参以 "--" 开头就 return true。
    if (args.empty() || args[0].starts_with("--"))
    {
        return true;
    }

    // 作用：有位置参数就默认走回放，省得再写 --replay。实现：先改模式，再按位置塞路径。
    config.mode = core::RunMode::Replay;
    config.video_path = args[0];
    // 作用：第二个位置是 IMU 文件，有就收。实现：size 检查后取下标 1。
    if (args.size() >= 2) config.imu_path = args[1];
    // 作用：第四个位置是输出目录，跳过第三个（历史遗留槽位）。实现：size 够 4 才取 args[3]。
    if (args.size() >= 4) config.output_directory = args[3];
    return true;
}

}   // namespace

// 作用：命令行总入口，把一堆字符串变成配置。实现：两遍扫描——先认位置参数，再逐项匹配长选项，出错即停。
CliResult parseCommandLine(int argc, char** argv)
{
    // 作用：结果容器，默认走回放模式。实现：默认构造后设 mode。
    CliResult result;
    result.config.mode = core::RunMode::Replay;

    // 作用：把 argv 包成 vector，后面按下标好操作。实现：跳过 argv[0] 程序名，构造区间。
    const std::vector<std::string> args(argv + 1, argv + argc);
    // 作用：先处理老式位置参数，失败就记录并打道回府。实现：调 applyLegacyArguments，假则写 error 返回。
    if (!applyLegacyArguments(args, result.config))
    {
        result.error = "无法解析位置参数";
        return result;
    }

    // 作用：逐项扫描所有参数，认得的就改配置。实现：普通 for 循环，需要值时借用下面的 lambda 前移下标。
    for (std::size_t i = 0; i < args.size(); ++i)
    {
        // 作用：当前参数和“后面还有没有值”的快照。实现：引用加一次下标比较。
        const std::string& arg = args[i];
        const bool has_value = i + 1 < args.size();

        // 作用：统一处理“这个开关需要跟一个值”的样板代码。实现：没值就写 error 返回空指针，有值就前移 i 并交出地址。
        auto requireValue = [&]() -> const std::string*
        {
            if (!has_value)
            {
                result.error = arg + " 缺少参数";
                return nullptr;
            }
            return &args[++i];
        };

        // 作用：不是 -- 开头的多半是位置参数，前面已经消费过，跳过。实现：starts_with 检查后 continue。
        if (!arg.starts_with("--"))
        {
            continue;
        }

        // 作用：--help / -h 只举手不干活，让 main 去打用法。实现：置 help 标志继续扫。
        if (arg == "--help" || arg == "-h")
        {
            result.help = true;
            continue;
        }
        // 作用：--list-cameras 同理，先记下，main 决定怎么处理。实现：置标志继续。
        if (arg == "--list-cameras")
        {
            result.list_cameras = true;
            continue;
        }
        // 作用：显式选回放模式。实现：改 mode 字段。
        if (arg == "--replay")
        {
            result.config.mode = core::RunMode::Replay;
            continue;
        }
        // 作用：显式选实时模式，后面会去开相机和串口。实现：改 mode 字段。
        if (arg == "--live")
        {
            result.config.mode = core::RunMode::Live;
            continue;
        }
        // 作用：指定回放视频路径。实现：requireValue 取值，缺值就 break。
        if (arg == "--video")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.video_path = *value;
            continue;
        }
        // 作用：指定 IMU 时间序列文件。实现：取值后写 imu_path。
        if (arg == "--imu")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.imu_path = *value;
            continue;
        }
        // 作用：指定回放输出目录，视频和 CSV 都往这丢。实现：取值后写 output_directory。
        if (arg == "--output")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.output_directory = *value;
            continue;
        }
        // 作用：指定串口设备，实时模式收反馈靠它。实现：取值后写 serial.device。
        if (arg == "--serial")
        {
            const auto* value = requireValue();
            if (!value) break;
            result.config.serial.device = *value;
            continue;
        }
        // 作用：设串口波特率，整数。实现：取值后 parseInt，解析失败写 error 并 break。
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
        // 作用：设通信延迟，单位秒，用来倒推板卡事件时间。实现：取值后 parseDouble 写 communication_latency。
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
        // 作用：选 MVS 相机索引。实现：parseUInt 后写 camera.device_index，负数或垃圾值直接报错。
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
        // 作用：设采集宽度，0 表示不改设备默认值。实现：parseInt 后写 camera.width。
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
        // 作用：设采集高度，0 表示不改。实现：parseInt 后写 camera.height。
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
        // 作用：设相机帧率，0 表示不改。实现：parseFloat 后写 camera.frame_rate。
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
        // 作用：设曝光时间，单位微秒，0 表示不改。实现：parseFloat 后写 exposure_time_us。
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
        // 作用：设相机增益，负数表示不改。实现：parseFloat 后写 camera.gain。
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
        // 作用：选时间源：host 用上位机收图时刻，device 用相机自带戳。实现：字符串二分，顺便把非法值当错误。
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
        // 作用：给帧时间加一个常数偏移，台架上标定用。实现：parseDouble 后写 timestamp_offset_us。
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
        // 作用：设曝光锚点，表示设备戳落在曝光区间的哪个比例。实现：解析后检查 0~1，越界直接报错。
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
        // 作用：按场地一次性套用曝光、增益、帧率三件套。实现：解析名字后调 applyExposurePreset。
        if (arg == "--exposure-preset")
        {
            const auto* value = requireValue();
            if (!value) break;

            core::ExposurePreset preset {};
            if (*value == "dark")
            {
                preset = core::ExposurePreset::Dark;
            }
            else if (*value == "indoor")
            {
                preset = core::ExposurePreset::Indoor;
            }
            else if (*value == "bright")
            {
                preset = core::ExposurePreset::Bright;
            }
            else
            {
                result.error =
                    "--exposure-preset 只支持 dark / indoor / bright: " +
                    *value;
                break;
            }

            core::applyExposurePreset(preset, result.config.camera);
            continue;
        }
        // 作用：关掉 Chunk-Exposure，相机不回报每帧曝光信息。实现：把 chunk_exposure 置假。
        if (arg == "--no-chunk-exposure")
        {
            result.config.camera.chunk_exposure = false;
            continue;
        }
        // 作用：打开相机时间日志，排查时间戳问题时用。实现：timing_log 置真。
        if (arg == "--camera-timing-log")
        {
            result.config.camera.timing_log = true;
            continue;
        }
        // 作用：没有云台反馈时也能跑，用固定姿态顶替。实现：imu_enabled 置假。
        if (arg == "--no-imu")
        {
            result.config.imu_enabled = false;
            continue;
        }
        // 作用：把原始观测量单独落盘，标定 R 用。实现：log_observations 置真。
        if (arg == "--log-observations")
        {
            result.config.log_observations = true;
            continue;
        }
        // 作用：加载外部标定文件，覆盖内置内参。实现：调 loadIntrinsicsFile，失败就把错误塞进 error 并停止解析。
        if (arg == "--calibration")
        {
            const auto* value = requireValue();
            if (!value) break;
            if (!loadIntrinsicsFile(
                    *value,
                    result.config.intrinsics,
                    result.error))
            {
                break;
            }
            continue;
        }
        // 作用：设送进 EKF 的最大目标距离，太远的当噪声扔掉。实现：parseDouble 后写 target_max_distance。
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
        // 作用：设 IMU 与图像的时间偏移，单位毫秒。实现：先按毫秒解析，再除 1000 存成秒。
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

        // 作用：谁都不认识就报错收工，别让人以为参数生效了。实现：写 error 后 break。
        result.error = "未知参数: " + arg;
        break;
    }

    return result;
}

// 作用：打印用法，告诉用户有哪些旋钮可拧。实现：一次性拼好长字符串丢进输出流，不改任何状态。
void printUsage(std::ostream& out)
{
    // 作用：把三段用法说明打包输出，省得逐行多次写流。实现：一个长字符串字面量直接 << 给 out。
    out <<
        "用法:\n"
        "  kalman_filter --replay [选项]\n"
        "  kalman_filter --live [选项]\n"
        "  kalman_filter --list-cameras\n\n"
        "通用选项:\n"
        "  --calibration PATH    相机标定文件（OpenCV YAML），不填用内置参考内参\n"
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
        "  --exposure-preset P   dark / indoor / bright，一次配好曝光+增益+帧率\n"
        "  --gain G              设置增益，负数表示不修改\n"
        "  --camera-time-source S  host 或 device，默认 device\n"
        "  --camera-time-offset-us US  帧时间常数偏移，台架标定用\n"
        "  --camera-exposure-anchor R  设备戳在曝光区间的比例，0~1，默认 0.5\n"
        "  --no-chunk-exposure   关闭 Chunk-Exposure，不读每帧曝光\n"
        "  --camera-timing-log   打印时钟校准结果和每帧时间戳\n"
        "  --no-imu              不用 C 板姿态，假定云台静止（串口可不接）\n"
        "  --log-observations    把每块装甲板的原始观测量写入 observations.csv\n"
        "  --serial PATH         串口设备\n"
        "  --baud N              串口波特率\n"
        "  --comm-latency S      通信延迟估计，秒\n"
        "  --max-distance M      送入 EKF 的最大目标距离\n";
}

}   // namespace app
