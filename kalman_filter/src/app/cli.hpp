#ifndef AUTOAIM_APP_CLI_HPP
#define AUTOAIM_APP_CLI_HPP

#include <iosfwd>
#include <string>

#include "core/config.hpp"

namespace app {

struct CliResult {
  // 作用：解析出来的完整配置，main 直接拿去跑。实现：先给默认值，再被各参数逐项覆盖。
  core::AppConfig config;
  // 作用：第一个撞上的解析错误，空串表示一切正常。实现：出错时写一句话并停止继续解析。
  std::string error;
  // 作用：用户是不是只想要帮助。实现：遇到 --help/-h 就置 true，main 优先处理。
  bool help{};
  // 作用：用户是不是想列相机。实现：遇到 --list-cameras 置 true，main 在启动前拦截。
  bool list_cameras{};
};

// 作用：把 argv 翻成配置，顺带处理帮助和错误。实现：先试位置参数，再逐项扫 -- 开头的开关，遇到第一个错误就收手。
// 参数：argc/argv —— 原样的命令行；argv[0] 是程序名，会被跳过。
// 返回：解析结果；error 非空表示失败，此时 config 里可能是半成品，别用。
CliResult parseCommandLine(int argc, char** argv);
// 作用：把用法说明打到指定流，stdout 和 stderr 都能用。实现：一次性输出预置的字符串，不读配置。
// 参数：out —— 目标输出流。返回：无。
void printUsage(std::ostream& out);

}  // namespace app

#endif
