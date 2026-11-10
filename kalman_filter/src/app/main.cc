# include "app/cli.hpp"
# include "app/mode_runner.hpp"

# include <iostream>

int main(int argc, char** argv)
{
    const app::CliResult cli =
        app::parseCommandLine(argc, argv);

    if (cli.help)
    {
        app::printUsage(std::cout);
        return 0;
    }

    if (!cli.error.empty())
    {
        std::cerr << cli.error << '\n';
        app::printUsage(std::cerr);
        return 2;
    }

    if (cli.list_cameras)
    {
        return app::listCameras();
    }

    if (cli.config.mode == core::RunMode::Live)
    {
        return app::runLive(cli.config);
    }
    return app::runReplay(cli.config);
}
