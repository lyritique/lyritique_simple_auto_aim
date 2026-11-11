# ifndef AUTOAIM_APP_MODE_RUNNER_HPP
# define AUTOAIM_APP_MODE_RUNNER_HPP

# include "core/config.hpp"

namespace app
{

// 作用：跑离线回放，把视频和 IMU 喂给算法并输出结果。实现：构造 ReplayPipeline 后一路 run，失败打印错误。
// 参数：config —— 已经解析好的配置。返回：进程退出码，0 成功、1 失败。
int runReplay(const core::AppConfig& config);
// 作用：跑实时链路，边收图边收反馈，Ctrl+C 收工。实现：起管线、挂信号、进主循环，结束后打统计。
// 参数：config —— 已经解析好的配置。返回：进程退出码，0 正常收尾、1 启动失败。
int runLive(const core::AppConfig& config);

// 列出已连接的 MVS 相机，给 --list-cameras 用。
// 作用：把相机枚举结果打成人类可读的清单。实现：调 MvsCamera::enumerate，逐台打印索引、型号和序列号。
// 参数：无。返回：进程退出码，0 表示枚举成功（哪怕一台没有），1 表示枚举出错。
int listCameras();

}   // namespace app

# endif
