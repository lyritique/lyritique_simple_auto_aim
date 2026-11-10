# ifndef AUTOAIM_APP_CLI_HPP
# define AUTOAIM_APP_CLI_HPP

# include <iosfwd>
# include <string>

# include "core/config.hpp"

namespace app
{

struct CliResult
{
    core::AppConfig config;
    std::string error;
    bool help {};
    bool list_cameras {};
};

CliResult parseCommandLine(int argc, char** argv);
void printUsage(std::ostream& out);

}   // namespace app

# endif
