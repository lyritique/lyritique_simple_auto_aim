#include "runtime/replay_pipeline.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <utility>

#include "debug/draw.hpp"
#include "runtime/frame_processor.hpp"
#include "sensors/video_source.hpp"
#include "timing/replay_imu.hpp"

namespace realtime {
namespace {

// 作用：把四元数变成旋转矩阵，顺手单位化防脏数据。实现：normalize 后 toRotationMatrix。
Eigen::Matrix3d toMatrix(const Eigen::Quaterniond& q) { return q.normalized().toRotationMatrix(); }

}  // namespace

// 作用：把配置收进来，回放时按它找文件和输出目录。实现：移动构造一个配置成员。
ReplayPipeline::ReplayPipeline(core::AppConfig config) : config_(std::move(config)) {}

// 作用：离线跑完整条链路，产出带框视频和状态 CSV。实现：准备工作全放在 try 里，任何一步失败都转成 error 字符串返回。
bool ReplayPipeline::run(std::string* error) {
  // 作用：离线批量处理，一处抛异常就整体收手。实现：整个函数体包一层 try/catch。
  try {
    // 作用：先把 IMU 序列读进来，后面每帧对齐都要用。实现：调 readImuFile，空数据直接判失败。
    const auto imu_data = replay::readImuFile(config_.imu_path);
    if (imu_data.empty()) {
      if (error) *error = "IMU 数据为空";
      return false;
    }

    // 作用：造一个视频源的相机抽象，和实时链路共用同一接口。实现：unique_ptr 持有 VideoSource，open 失败就返回。
    std::unique_ptr<camera::CameraSource> source = std::make_unique<camera::VideoSource>(config_.video_path);
    if (!source->open(error)) return false;

    // 作用：拿视频元信息，帧率决定输出视频的时间基。实现：source->info() 后检查帧率有效性。
    const camera::SourceInfo source_info = source->info();
    if (source_info.frame_rate <= 0.0) {
      if (error) *error = "视频帧率无效";
      return false;
    }

    // 作用：确保输出目录存在，不然写文件会失败。实现：create_directories 带 error_code，失败了再报。
    std::error_code fs_error;
    std::filesystem::create_directories(config_.output_directory, fs_error);
    if (fs_error) {
      if (error) {
        *error = "无法创建输出目录: " + fs_error.message();
      }
      return false;
    }

    // 作用：拼出输出视频路径，固定叫 output_video.avi。实现：输出目录加文件名。
    const std::string video_output = config_.output_directory + "/output_video.avi";
    // 作用：开视频写入器，沿用源视频的帧率和分辨率。实现：MJPEG 编码，构造后检查 isOpened。
    cv::VideoWriter writer(video_output, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), source_info.frame_rate,
                           cv::Size(source_info.width, source_info.height));
    if (!writer.isOpened()) {
      if (error) *error = "无法创建输出视频: " + video_output;
      return false;
    }

    // 作用：开 CSV 记录每帧状态，方便后处理画曲线。实现：ofstream 打开，失败即返回。
    std::ofstream csv(config_.output_directory + "/data.csv");
    if (!csv) {
      if (error) *error = "csv 无法创建";
      return false;
    }
    // 作用：写表头，列名就是后面要填的状态顺序。实现：先写一行固定的逗号分隔字段。
    csv << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r\r\n";

    // 作用：每帧都要用它跑检测和滤波，循环外建一次省得反复构造。实现：局部 FrameProcessor。
    FrameProcessor processor(config_);

    // 作用：视频起始时刻取自 IMU 第一帧时间，两条时间轴从此对齐。实现：直接取 imu_data.front().time。
    const double video_start_time = imu_data.front().time;
    // 作用：拿起始时刻的姿态当世界系基准。实现：先按偏移对齐 IMU，再转成矩阵存成 R_0。
    const auto imu_0 = replay::alignImuToImage(imu_data, video_start_time, config_.timing.replay_imu_offset_seconds);
    const Eigen::Matrix3d R_0 = toMatrix(imu_0.q);

    // 作用：复用同一帧对象，减少每轮构造。实现：循环外声明。
    core::TimestampedFrame source_frame;
    // 作用：一帧一帧读到视频末尾。实现：无限循环里靠 FrameReadStatus 决定继续还是退出。
    for (;;) {
      // 作用：从视频源取下一帧。实现：read 返回状态码，End 就跳出。
      const auto status = source->read(source_frame, error);
      if (status == core::FrameReadStatus::End) break;
      // 作用：读帧失败要留个明白话，别让人猜。实现：状态非 Ok 时补错误信息再返回。
      if (status != core::FrameReadStatus::Ok) {
        if (!error || error->empty()) {
          if (error) *error = "视频读取失败";
        }
        return false;
      }

      // 作用：拿一帧彩色图，后面画框都画在它上面。实现：浅拷贝 source_frame.bgr 的 Mat 头。
      cv::Mat frame = source_frame.bgr;
      // 作用：把视频相对时间换成 IMU 的绝对时间。实现：起始时间加帧时间戳。
      const double image_time = video_start_time + source_frame.timestamp;
      // 作用：取该时刻的 IMU 姿态，同样按配置补一个固定偏移。实现：alignImuToImage。
      const auto imu_k = replay::alignImuToImage(imu_data, image_time, config_.timing.replay_imu_offset_seconds);
      // 作用：算从基准姿态到当前姿态的相对旋转，给算法世界用。实现：R_0 转置乘当前旋转。
      const Eigen::Matrix3d R = R_0.transpose() * toMatrix(imu_k.q);

      // 作用：这一帧正式进入检测和滤波流水线。实现：processor.process 带帧和旋转矩阵。
      const FrameProcessingResult processing = processor.process(source_frame, R);
      // 作用：把检测框画到图上，肉眼验证检测效果。实现：drawDetections。
      draw::drawDetections(frame, processing.detections);

      // 作用：原始角点画黄色，细化角点画紫色，方便对比亚像素效果。实现：两层循环各画一遍圆。
      for (const auto& observation : processing.observations) {
        for (const auto& point : observation.detection.corners) {
          cv::circle(frame, point, 2, cv::Scalar(0, 255, 255), 10);
        }
        for (const auto& point : observation.refined_corners) {
          cv::circle(frame, point, 2, cv::Scalar(255, 0, 255), 10);
        }
      }

      // 作用：给滤波结果起个短名字，后面反复用。实现：常量引用 processing.state。
      const auto& output = processing.state;

      // 作用：状态可信就画四面装甲板的重投影，检验位姿准不准。实现：valid 时调 drawFourArmorReprojections。
      if (output.valid) {
        draw::drawFourArmorReprojections(frame, output, processor.cameraMatrix(), processor.distortionCoeffs(), false);
      }

      // 作用：把本帧结果写进 CSV，无效帧单独标注。实现：invalid 就写“偏离过大”，否则写时间、9 维状态和重投影误差。
      if (!output.valid) {
        csv << image_time << "偏离过大" << "\r\n";
      } else {
        csv << image_time << ',' << output.valid << ',';
        for (int i = 0; i < 9; ++i) {
          csv << output.state(i) << ',';
        }
        for (const double value : processing.reprojection_errors) {
          csv << value << ',';
        }
        csv << "\r\n";
      }

      // 作用：把状态文字和这一帧一起写进输出视频。实现：先 drawStateText，再 writer.write。
      draw::drawStateText(frame, output);
      writer.write(frame);
    }

    // 作用：收尾释放资源，保证文件落盘。实现：关 CSV、释放写入器、关视频源。
    csv.close();
    writer.release();
    source->close();
    return true;
  }
  // 作用：把异常转成普通错误返回，保持接口不抛。实现：catch std::exception，写 error 后返回 false。
  catch (const std::exception& exception) {
    if (error) *error = exception.what();
    return false;
  }
}

}  // namespace realtime
