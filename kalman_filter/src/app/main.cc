# include "app/cli.hpp"
# include "app/mode_runner.hpp"

# include <iostream>

// 作用：程序入口，决定这一次到底要干什么。实现：先解析命令行，再按 help/错误/列相机/模式四个分支依次分派。
int main(int argc, char** argv)
{
    // 作用：拿到解析结果，后面全看它。实现：调 parseCommandLine，参数原样转交。
    const app::CliResult cli =
        app::parseCommandLine(argc, argv);

    // 作用：用户只要帮助就打印用法，别真去开设备。实现：help 优先，输出到 stdout 后正常退出。
    if (cli.help)
    {
        app::printUsage(std::cout);
        return 0;
    }

    // 作用：参数有毛病就骂两句并给用法，返回 2 区分于运行失败。实现：error 非空时写 stderr，再打用法。
    if (!cli.error.empty())
    {
        std::cerr << cli.error << '\n';
        app::printUsage(std::cerr);
        return 2;
    }

    // 作用：只想看相机就列完走人，不启动算法。实现：转调 listCameras 并用它的退出码。
    if (cli.list_cameras)
    {
        return app::listCameras();
    }

    // 作用：按模式选一条路：实时或回放。实现：比较 mode 字段后分别调用对应 runner。
    if (cli.config.mode == core::RunMode::Live)
    {
        return app::runLive(cli.config);
    }
    return app::runReplay(cli.config);
}
