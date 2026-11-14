#ifndef AUTOAIM_RUNTIME_REPLAY_PIPELINE_HPP
#define AUTOAIM_RUNTIME_REPLAY_PIPELINE_HPP

#include <string>

#include "core/config.hpp"

namespace realtime {

class ReplayPipeline {
 public:
  // 作用：把离线回放配置存起来，后面按视频和 IMU 两条数据源跑。实现：移动构造配置，除此之外啥也不干。
  // 参数：config —— 全量配置，按值传入；只用到 video_path、imu_path、output_directory 和时间偏移。
  explicit ReplayPipeline(core::AppConfig config = {});

  // 作用：从视频逐帧回放，配上 IMU 姿态跑完整算法，输出画好框的视频和 CSV。实现：读 IMU、开视频、逐帧对齐处理并写文件。
  // 参数：error —— 可选出错信息出口，失败原因会写进去。
  // 返回：true 表示视频处理完且文件已落盘；false 表示中途某步失败，具体看 error。
  bool run(std::string* error = nullptr);

  // 作用：只读配置，外部查参数用。实现：内联返回常量引用，不拷贝。
  // 参数：无。返回：构造时存下的配置引用，跟随对象生命周期。
  const core::AppConfig& config() const noexcept { return config_; }

 private:
  // 作用：回放所需的全部参数：视频路径、IMU 路径、输出目录、时间偏移。实现：值成员，构造时移入，之后只读。
  core::AppConfig config_;
};

}  // namespace realtime

#endif
