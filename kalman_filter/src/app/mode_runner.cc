# include "app/mode_runner.hpp"

# include "runtime/realtime_pipeline.hpp"
# include "runtime/replay_pipeline.hpp"
# include "sensors/mvs_camera.hpp"

# include <atomic>
# include <csignal>
# include <iostream>
# include <string>
# include <vector>

namespace app
{
namespace
{

// 作用：给信号处理函数留一个“往哪写停止标志”的地址。实现：全局裸指针，只有 runLive 期间才非空。
std::atomic_bool* g_stop_requested {};

//按 C 语言的规则来生成符号名(extern "C"),从而能被 C 接口(signal)正确识别和调用
// 作用：Ctrl+C 或 kill 时把主循环叫停，顺便只做异步信号安全的事。实现：extern "C" 保住 C 链接，只对 atomic 做一次 store。
extern "C" void requestStop(int)
{
    if (g_stop_requested != nullptr)
    {
        g_stop_requested->store(true);
    }
}

}   // namespace

// 作用：离线回放入口，跑完就退出。实现：构造 ReplayPipeline 调 run，失败把错误打出来并返回 1。
int runReplay(const core::AppConfig& config)
{
    // 作用：建管线，配置原样传入。实现：栈上 ReplayPipeline 对象。
    realtime::ReplayPipeline pipeline(config);
    // 作用：准备错误槽，失败时接住原因。实现：局部 string 的地址交给 run。
    std::string error;
    // 作用：跑不起来就报错，拒绝默默失败。实现：run 返回假时写 stderr 并返回 1。
    if (!pipeline.run(&error))
    {
        std::cerr << "回放失败: " << error << '\n';
        return 1;
    }
    return 0;
}

// 作用：实时模式入口，负责起管线、挂信号、跑主循环、收尾打统计。实现：start → 注册 SIGINT/SIGTERM → run → 还原信号 → 打印计数。
int runLive(const core::AppConfig& config)
{
    // 作用：建实时管线，配置原样传入。实现：栈上 RealtimePipeline 对象。
    realtime::RealtimePipeline pipeline(config);
    // 作用：没有云台反馈时把前提说清楚，免得把结果当成真实标定结果。实现：imu_enabled 为假就提示一句。
    if (!config.imu_enabled)
    {
        std::cerr
            << "警告: 无 IMU 模式，假定云台静止；世界系固定在初始相机姿态。\n";
    }
    // 作用：准备错误槽并尝试启动设备。实现：start 失败就打印原因并返回 1。
    std::string error;
    if (!pipeline.start(&error))
    {
        std::cerr << "实时模式启动失败: " << error << '\n';
        return 1;
    }

    // 作用：造一个本地的停止标志，信号处理函数会把它置真。实现：atomic bool，初值 false。
    std::atomic_bool stop_requested {false};
    // 作用：把标志地址曝光给信号处理函数，并接管两个终止信号。实现：全局指针赋值后 signal 注册。
    g_stop_requested = &stop_requested;
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    // 作用：进主循环，直到有人喊停。实现：把停止标志交给 run，它自己管读帧和退出。
    pipeline.run(stop_requested);

    // 作用：收工后把全局指针和信号处理还原，别影响后续代码。实现：指针置空，两个 signal 恢复 SIG_DFL。
    g_stop_requested = nullptr;
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);

    // 作用：打一份流水账，看看收了多少、丢了多少。实现：取 stats 快照后一次性输出四个计数。
    const auto stats = pipeline.stats();
    std::cout << "实时模式结束: frames=" << stats.processed_frames
              << " valid=" << stats.valid_frames
              << " dropped=" << stats.dropped_frames
              << " feedback=" << stats.received_feedback
              << '\n';
    return 0;
}

// 作用：列相机，给用户挑 --camera-index 用。实现：枚举设备，出错报错，空列表给排查提示，否则逐台打印信息。
int listCameras()
{
    // 作用：错误槽和结果容器。实现：先声明 string，枚举结果直接接返回值。
    std::string error;
    const std::vector<camera::CameraInfo> devices =
        camera::MvsCamera::enumerate(&error);

    // 作用：枚举本身报错就别继续装了。实现：error 非空时写 stderr 并返回 1。
    if (!error.empty())
    {
        std::cerr << "枚举相机失败: " << error << '\n';
        return 1;
    }

    // 作用：一台都没有时给点排查方向，比干瞪眼强。实现：empty 检查后打印提示，返回 0 表示流程正常。
    if (devices.empty())
    {
        std::cout << "没找到相机。检查一下 USB3 线/网线、MVS 驱动和相机供电。\n";
        return 0;
    }

    // 作用：先报总数，再逐台列细节。实现：输出 size，然后 range-for 遍历。
    std::cout << "找到 " << devices.size() << " 台相机:\n";
    for (const auto& device : devices)
    {
        // 作用：打印索引和型号，型号缺失就写“未知型号”。实现：索引加三目表达式。
        std::cout << "  [" << device.index << "] "
                  << (device.model.empty() ? "未知型号" : device.model);

        // 作用：有序列号就带上，多台同型号时靠它区分。实现：非空才输出。
        if (!device.serial.empty())
        {
            std::cout << "  SN:" << device.serial;
        }
        // 作用：有用户自定义名字也一并显示。实现：非空才输出。
        if (!device.user_name.empty())
        {
            std::cout << "  名字:" << device.user_name;
        }
        // 作用：显示传输方式，排查 USB/网口问题有用。实现：非空才输出。
        if (!device.transport.empty())
        {
            std::cout << "  传输:" << device.transport;
        }
        std::cout << '\n';
    }

    // 作用：告诉用户下一步怎么选。实现：打印一行固定提示后返回 0。
    std::cout << "用 --camera-index N 选一台。\n";
    return 0;
}

}   // namespace app
