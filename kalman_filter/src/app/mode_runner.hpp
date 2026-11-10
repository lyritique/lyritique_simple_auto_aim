# ifndef AUTOAIM_APP_MODE_RUNNER_HPP
# define AUTOAIM_APP_MODE_RUNNER_HPP

# include "core/config.hpp"

namespace app
{

int runReplay(const core::AppConfig& config);
int runLive(const core::AppConfig& config);

// 列出已连接的 MVS 相机，给 --list-cameras 用。
int listCameras();

}   // namespace app

# endif
