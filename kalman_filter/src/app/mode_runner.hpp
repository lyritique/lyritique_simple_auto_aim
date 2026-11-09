# ifndef AUTOAIM_APP_MODE_RUNNER_HPP
# define AUTOAIM_APP_MODE_RUNNER_HPP

# include "core/config.hpp"

namespace app
{

int runReplay(const core::AppConfig& config);
int runLive(const core::AppConfig& config);

}   // namespace app

# endif
