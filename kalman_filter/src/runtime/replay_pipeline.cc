#include "runtime/replay_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
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
ReplayPipeline::ReplayPipeline(core::AppConfig config)
    : config_(std::move(config)),
      aim_(control::AimConfig{config_.control.bullet_speed,
                              config_.control.yaw_sign,
                              config_.control.pitch_sign,
                              config_.control.yaw_offset_deg * 0.017453292519943295,
                              config_.control.pitch_offset_deg * 0.017453292519943295,
                              config_.control.cam_remap,
                              config_.control.up_axis == 1,
                              config_.control.compensate_drop,
                              config_.control.gravity}) {}

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
    csv << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r,dr,dz\r\n";

    // 作用：需要标定观测噪声时，把每块板的原始观测也落盘，字段和实时模式一致。
    // 实现：受 log_observations 控制，输出 observations.csv。
    std::ofstream observation_csv;
    if (config_.log_observations) {
      observation_csv.open(config_.output_directory + "/observations.csv");
      if (observation_csv) {
        observation_csv << std::fixed << std::setprecision(9);
        observation_csv << "time,azi,ele,dist,yaw,reproj_err,sent_to_ekf\r\n";
      }
    }

    // 作用：每帧都要用它跑检测和滤波，循环外建一次省得反复构造。实现：局部 FrameProcessor。
    FrameProcessor processor(config_);

    // 作用：视频起始时刻取自 IMU 第一帧时间，两条时间轴从此对齐。实现：直接取 imu_data.front().time。
    const double video_start_time = imu_data.front().time;
    // 作用：拿起始时刻的姿态当世界系基准。实现：先按偏移对齐 IMU，再转成矩阵存成 R_0。
    const auto imu_0 = replay::alignImuToImage(imu_data, video_start_time, config_.timing.replay_imu_offset_seconds);
    const Eigen::Matrix3d R_0 = toMatrix(imu_0.q);

    // 作用：复用同一帧对象，减少每轮构造。实现：循环外声明。
    core::TimestampedFrame source_frame;

    // 作用：回放各段耗时累计器，够 50 帧打一行。实现：循环外局部变量，只有本线程读写。
    double prof_read = 0.0;
    double prof_imu = 0.0;
    double prof_process = 0.0;
    double prof_draw = 0.0;
    double prof_csv = 0.0;
    double prof_display = 0.0;
    double prof_encode = 0.0;
    double prof_cycle = 0.0;
    int prof_frames = 0;

    // 作用：一帧一帧读到视频末尾。实现：无限循环里靠 FrameReadStatus 决定继续还是退出。
    for (;;) {
      const auto cycle_start = std::chrono::steady_clock::now();
      // 作用：从视频源取下一帧。实现：read 返回状态码，End 就跳出。
      const auto status = source->read(source_frame, error);
      const auto read_done = std::chrono::steady_clock::now();
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
      const auto imu_start = std::chrono::steady_clock::now();
      const auto imu_k = replay::alignImuToImage(imu_data, image_time, config_.timing.replay_imu_offset_seconds);
      // 作用：算从基准姿态到当前姿态的相对旋转，给算法世界用。实现：R_0 转置乘当前旋转。
      const Eigen::Matrix3d R = R_0.transpose() * toMatrix(imu_k.q);
      const auto imu_done = std::chrono::steady_clock::now();

      // 作用：这一帧正式进入检测和滤波流水线。实现：processor.process 带帧和旋转矩阵。
      const auto process_start = std::chrono::steady_clock::now();
      const FrameProcessingResult processing = processor.process(source_frame, R);
      const auto process_done = std::chrono::steady_clock::now();

      const auto draw_start = std::chrono::steady_clock::now();
      double csv_ms = 0.0;
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
        if (observation_csv) {
          const auto csv_start = std::chrono::steady_clock::now();
          observation_csv << image_time << ',' << observation.ypd.azi << ',' << observation.ypd.ele << ','
                          << observation.ypd.dist << ',' << observation.ekf_yaw << ',' << observation.pose.err << ','
                          << (observation.sent_to_ekf ? 1 : 0) << "\r\n";
          csv_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - csv_start).count();
        }
      }

      // 作用：给滤波结果起个短名字，后面反复用。实现：常量引用 processing.state。
      const auto& output = processing.state;

      // 作用：状态可信就画四面装甲板的重投影，检验位姿准不准。实现：valid 时调 drawFourArmorReprojections。
      if (output.valid) {
        draw::drawFourArmorReprojections(frame, output, R, processor.cameraMatrix(), processor.distortionCoeffs(), false);
      }
      const double draw_block_ms =
          std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - draw_start).count();
      double draw_ms = std::max(0.0, draw_block_ms - csv_ms);

      // 作用：离线验证瞄准解算，只看角度跟不跟得上，不发任何东西。实现：0.5 s 打一行。
      if (config_.control.debug_log && output.valid &&
          (last_aim_log_time_ < 0.0 || image_time - last_aim_log_time_ >= 0.5)) {
        last_aim_log_time_ = image_time;
        const Eigen::Quaterniond q_init(R_0);
        const Eigen::Quaterniond q_now(imu_k.q);
        const auto aim = aim_.solve(output, q_init, q_now, config_.serial.communication_latency, 0.0, image_time);
        constexpr double kRad2Deg = 57.29577951308232;
        if (aim.valid) {
          std::clog << std::fixed << std::setprecision(2) << "[aim] t=" << image_time << " yaw=" << aim.yaw * kRad2Deg
                    << "° pitch=" << aim.pitch * kRad2Deg << "° id=" << aim.armor_id << " dist=" << aim.distance
                    << "m fly=" << aim.flight_time * 1e3 << "ms\n";
        }
      }

      // 作用：把本帧结果写进 CSV，无效帧单独标注。实现：invalid 就写“偏离过大”，否则写时间、11 维状态和重投影误差。
      const auto state_csv_start = std::chrono::steady_clock::now();
      if (!output.valid) {
        csv << image_time << "偏离过大" << "\r\n";
      } else {
        csv << image_time << ',' << output.valid << ',';
        for (int i = 0; i < 11; ++i) {
          csv << output.state(i) << ',';
        }
        for (const double value : processing.reprojection_errors) {
          csv << value << ',';
        }
        csv << "\r\n";
      }
      csv_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - state_csv_start).count();

      // 作用：把状态文字和这一帧一起写进输出视频。实现：先 drawStateText，再 writer.write。
      const auto state_text_start = std::chrono::steady_clock::now();
      draw::drawStateText(frame, output);
      draw_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - state_text_start).count();

      // 作用：调试开关打开就把画完四面装甲板重投影的画面弹到屏幕上。实现：imshow + waitKey(1) 泵一下 GUI 事件。
      double display_ms = 0.0;
      if (config_.debug_draw) {
        const auto display_start = std::chrono::steady_clock::now();
        cv::imshow("debug draw", frame);
        cv::waitKey(1);
        display_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - display_start).count();
      }

      const auto encode_start = std::chrono::steady_clock::now();
      writer.write(frame);
      const auto cycle_done = std::chrono::steady_clock::now();

      if (config_.profile) {
        const auto to_ms = [](const auto& a, const auto& b) {
          return std::chrono::duration<double, std::milli>(b - a).count();
        };
        const double read_ms = to_ms(cycle_start, read_done);
        const double imu_ms = to_ms(imu_start, imu_done);
        const double process_ms = to_ms(process_start, process_done);
        const double encode_ms = to_ms(encode_start, cycle_done);
        prof_read += read_ms;
        prof_imu += imu_ms;
        prof_process += process_ms;
        prof_draw += draw_ms;
        prof_csv += csv_ms;
        prof_display += display_ms;
        prof_encode += encode_ms;
        prof_cycle += to_ms(cycle_start, cycle_done);
        if (++prof_frames >= 50) {
          const double n = static_cast<double>(prof_frames);
          const double cycle_avg = prof_cycle / n;
          const double accounted =
              (prof_read + prof_imu + prof_process + prof_draw + prof_csv + prof_display + prof_encode) / n;
          std::clog << "[replay] cycle=" << cycle_avg << "ms read=" << prof_read / n << " imu=" << prof_imu / n
                    << " process=" << prof_process / n << " draw=" << prof_draw / n << " csv=" << prof_csv / n
                    << " display=" << prof_display / n << " encode=" << prof_encode / n
                    << " other=" << std::max(0.0, cycle_avg - accounted) << "ms\n";
          prof_read = prof_imu = prof_process = prof_draw = 0.0;
          prof_csv = prof_display = prof_encode = prof_cycle = 0.0;
          prof_frames = 0;
        }
      }
    }

    // 作用：调试窗口用完得拆掉，不然进程退出后窗口还挂着。实现：开关打开时 destroyWindow。
    if (config_.debug_draw) {
      cv::destroyWindow("debug draw");
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
