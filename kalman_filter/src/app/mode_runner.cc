# include "app/mode_runner.hpp"

# include "runtime/realtime_pipeline.hpp"
# include "runtime/replay_pipeline.hpp"

# include <atomic>
# include <csignal>
# include <iostream>
# include <string>

namespace app
{
namespace
{

std::atomic_bool* g_stop_requested {};

extern "C" void requestStop(int)
{
    if (g_stop_requested != nullptr)
    {
        g_stop_requested->store(true);
    }
}

}   // namespace

int runReplay(const core::AppConfig& config)
{
    realtime::ReplayPipeline pipeline(config);
    std::string error;
    if (!pipeline.run(&error))
    {
        std::cerr << "回放失败: " << error << '\n';
        return 1;
    }
    return 0;
}

int runLive(const core::AppConfig& config)
{
    realtime::RealtimePipeline pipeline(config);
    std::string error;
    if (!pipeline.start(&error))
    {
        std::cerr << "实时模式启动失败: " << error << '\n';
        return 1;
    }

    std::atomic_bool stop_requested {false};
    g_stop_requested = &stop_requested;
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    pipeline.run(stop_requested);

    g_stop_requested = nullptr;
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);

    const auto stats = pipeline.stats();
    std::cout << "实时模式结束: frames=" << stats.processed_frames
              << " valid=" << stats.valid_frames
              << " dropped=" << stats.dropped_frames
              << " feedback=" << stats.received_feedback
              << '\n';
    return 0;
}

}   // namespace app
