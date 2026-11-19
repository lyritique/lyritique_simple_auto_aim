#include "runtime/realtime_pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <utility>

#include "debug/draw.hpp"
#include "sensors/mvs_camera.hpp"

namespace realtime {
namespace {

constexpr double kRad2Deg = 57.29577951308232;

// 作用：拿当前 steady_clock 的秒数，全链路统一的时间尺。实现：取 time_since_epoch 再转成 double 秒。
double steadyNowSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 作用：把 C 板自报的 yaw/pitch 和四元数解出来的对一下。
//       C 板的 yaw/pitch 是它体轴 +x（光轴方向）的方位角和仰角（世界 z 朝天），
//       所以这里必须取 UnitX；取 UnitZ 是相机系的轴，跟它对不上。
void logAttitudeConvention(const kalfilter::GimbalSample& attitude) {
  const Eigen::Vector3d axis = attitude.q * Eigen::Vector3d::UnitX();
  std::clog << "board yaw=" << attitude.yaw * kRad2Deg << "° pitch=" << attitude.pitch * kRad2Deg
            << "° | 体轴+x az=" << std::atan2(axis.y(), axis.x()) * kRad2Deg << "° el="
            << std::asin(std::clamp(axis.z(), -1.0, 1.0)) * kRad2Deg << "°";
}

// C 板世界系（x 前、y 左、z 上）-> EKF 水平世界 E（x 右、y 下、z 前）。
// E 的 y 是真正的竖直方向，四块装甲板「同一高度」的模型才成立。
Eigen::Matrix3d levelToBoardWorld() {
  Eigen::Matrix3d R;
  R << 1.0, 0.0, 0.0,   // E.x = W.x
      0.0, 0.0, 1.0,    // E.y = -W.z（向下）
      0.0, -1.0, 0.0;   // E.z = W.y
  return R;
}

// 用 C 板的 pitch/yaw 构造「相机系 -> 水平世界 E」的旋转。
// 约定（与 logAttitudeConvention / --up-axis z 一致）：
//   光轴在 C 板世界系里是 (cos p cos y, cos p sin y, sin p)，pitch 抬头为正。
// 只靠光轴方向定两个自由度，假设相机没有 roll（两轴云台）。
Eigen::Matrix3d cameraToLevelWorld(double pitch, double yaw) {
  const Eigen::Vector3d axis(std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch));

  const Eigen::Vector3d world_down(0.0, 0.0, -1.0);
  Eigen::Vector3d down = world_down - world_down.dot(axis) * axis;
  if (down.norm() < 1e-6) {
    // 光轴正对天/地，roll 不可观测，退化为单位阵，至少不产生 NaN。
    return Eigen::Matrix3d::Identity();
  }
  down.normalize();

  // 相机系：x 右、y 下、z 前；x = y × z。
  const Eigen::Vector3d right = down.cross(axis);

  Eigen::Matrix3d R_WC;
  R_WC.col(0) = right;
  R_WC.col(1) = down;
  R_WC.col(2) = axis;

  return levelToBoardWorld().transpose() * R_WC;
}

}  // namespace

// 作用：组装整条实时链路，把配置分发到各组件。实现：移动配置，用配置里的延迟、历史时长、帧率参数逐个构造成员。
RealtimePipeline::RealtimePipeline(RealtimePipelineConfig config)
    : config_(std::move(config)),
      clock_(config_.serial.communication_latency),
      history_(config_.timing.max_history_seconds, config_.timing.max_attitude_gap_seconds,
               config_.timing.max_attitude_prediction_seconds),
      processor_(config_),
      aim_(control::AimConfig{config_.control.bullet_speed,
                              config_.control.yaw_sign,
                              config_.control.pitch_sign,
                              config_.control.yaw_offset_deg * 0.017453292519943295,
                              config_.control.pitch_offset_deg * 0.017453292519943295,
                              config_.control.cam_remap,
                              config_.control.up_axis == 1,
                              config_.control.compensate_drop,
                              config_.control.gravity}) {}

// 作用：对象销毁时别把设备晾着。实现：转调 stop() 收拾线程和串口。
RealtimePipeline::~RealtimePipeline() { stop(); }

// 作用：把链路从冷启动拉到运行态。实现：先全部复位，再开串口、开相机，最后起反馈线程；中途失败原路回滚。
bool RealtimePipeline::start(std::string* error) {
  // 作用：防止重复启动导致两套线程和设备打架。实现：先 stop 一次，再清错误信息。
  stop();
  last_error_.clear();

  // 作用：清掉上一轮的时钟偏移，重新标定。实现：reset 后重新灌入配置里的通信延迟。
  clock_.reset();
  clock_.setCommLatency(config_.serial.communication_latency);
  // 作用：清空云台历史和延迟统计，就当第一次开机。实现：两个 clear，再把通信延迟当第一条样本记进去。
  history_.clear();
  latency_.clear();
  latency_.recordCommunication(config_.serial.communication_latency);

  // 作用：重置滤波器、初始姿态和反馈时间。实现：processor_.reset() 加三个成员归位。
  processor_.reset();
  has_q_init_ = false;
  q_init_.setIdentity();
  has_level_log_ = false;
  last_feedback_receive_time_.store(0.0);
  aim_.reset();
  last_aim_log_time_ = -1.0;
  send_error_reported_ = false;
  has_feedback_log_.store(false);

  // 作用：计数器全部归零，统计从新的一段开始。实现：锁内整体赋值一个空 PipelineStats。
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    stats_ = PipelineStats{};
  }

  // 作用：无 IMU 模式下串口整块跳过，姿态由固定值顶替。实现：整个打开流程包在 imu_enabled 判断里。
  if (config_.imu_enabled) {
    // 作用：串口路径都没配就别往下走了。实现：empty 检查后调 fail 返回。
    if (config_.serial.device.empty()) {
      return fail("串口设备路径为空", error);
    }

    // 作用：打开串口，反馈线程就靠它吃饭。实现：SerialLink::open 带设备路径和波特率，失败就报错。
    if (!serial_.open(config_.serial.device.c_str(), config_.serial.baud)) {
      return fail("串口打开失败: " + config_.serial.device + " @" + std::to_string(config_.serial.baud) + " -> " +
                      serial_.lastError(),
                  error);
    }
    // P0：64 B/帧、8N1（每字节 10 bit）下的物理上限，控制频率不能拍脑袋定。
    const int max_frame_hz = config_.serial.baud / (static_cast<int>(comm::kFrameSize) * 10);
    std::clog << "[串口] " << config_.serial.device << " @" << config_.serial.baud << " 8N1，"
              << comm::kFrameSize << " B/帧 => 理论上限 " << max_frame_hz << " 帧/s\n";
  }

  // 作用：按配置造出具体相机（实时是 MVS）。实现：make_unique 后调 open，失败就把刚开的串口也关掉。
  camera_ = std::make_unique<camera::MvsCamera>(config_.camera);
  if (!camera_->open(error)) {
    serial_.close();
    camera_->close();
    return false;
  }

  // 作用：设备都就绪了，才正式宣布运行，再起收包线程。实现：先开 CSV，再置 running_，最后起 feedback 线程。
  openCsv();
  frame_callback_ = {};
  running_.store(true);
  // 作用：无 IMU 模式没有反馈可收，就别起线程。实现：imu_enabled 为真才构造 jthread。
  if (config_.imu_enabled) {
    feedback_thread_ = std::jthread([this](std::stop_token token) { feedbackLoop(token); });
  }
  return true;
}

// 作用：主入口。实现：异步走 capture/perception 两线程，同步走内联，结束后统一收摊。
void RealtimePipeline::run(const std::atomic_bool& external_stop, FrameCallback callback) {
  // 作用：没 start 就别空转。实现：running_ 为假直接返回。
  if (!running_.load()) return;

  if (config_.pipeline.async) {
    runAsyncLoop(external_stop, std::move(callback));
  } else {
    runSyncLoop(external_stop, callback);
  }
  stop();
}

// 作用：同步内联路径，读帧、处理、落盘、画图都在当前线程。
void RealtimePipeline::runSyncLoop(const std::atomic_bool& external_stop, const FrameCallback& callback) {
  // 作用：分段计时累加器，把「等帧 / 处理 / 落盘」拆开看。实现：单线程局部变量，够 50 帧打一次。
  double prof_read = 0.0;
  double prof_process = 0.0;
  double prof_write = 0.0;
  double prof_cycle = 0.0;
  int prof_frames = 0;

  // P0：processFrame 内部的细分耗时，和 prof_process 一起每 50 帧打一行。
  double prof_wait = 0.0;
  double prof_detect = 0.0;
  double prof_pnp = 0.0;
  double prof_ekf = 0.0;
  double prof_aim = 0.0;
  double prof_acq = 0.0;

  // 作用：画图耗时单独累计，别让 imshow 的等待混进 process 计时里。实现：单线程局部变量，够 50 帧打一行。
  double draw_total_ms = 0.0;
  int draw_frames = 0;

  // 作用：两个停止条件都要看：自己不想跑了，或外部喊停。实现：while 同时检查 running_ 和 external_stop。
  while (running_.load() && !external_stop.load()) {
    const auto cycle_start = std::chrono::steady_clock::now();
    // 作用：准备一个空帧和错误槽，交给相机去填。实现：栈上默认构造。
    core::TimestampedFrame frame;
    std::string error;
    // 作用：向相机要一帧。实现：read 返回状态码，后续分支处理。
    const auto status = camera_->read(frame, &error);
    const auto read_done = std::chrono::steady_clock::now();
    // P0：收帧时刻转成和全链路一致的秒数，用来算采集段延迟。
    const double read_done_time = std::chrono::duration<double>(read_done.time_since_epoch()).count();
    // 作用：暂时没图不算错误，下一轮再要。实现：Timeout 就 continue，不计数。
    if (status == core::FrameReadStatus::Timeout) continue;

    // 作用：真读坏了才计丢帧并留错误，睡 5ms 防止忙等刷屏。实现：锁内自增 dropped_frames，记错误，再 sleep。
    if (status != core::FrameReadStatus::Ok) {
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.dropped_frames;
      }
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = std::move(error);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 统计口径和异步一致：成功读到的帧都算 captured。
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.captured_frames;
    }

    // 作用：好帧就进入加工流水线。实现：processFrame 只算算法，落盘/画图由这里负责。
    PipelineFrameResult result = processFrame(frame, read_done_time);
    const auto process_done = std::chrono::steady_clock::now();

    // 作用：同步路径下 CSV 由当前线程写，保证每帧一行。
    writeCsv(result);

    // 作用：开了调试画图就在这里画，画图耗时单独统计，不混进算法耗时。
    if (config_.debug_draw) {
      result.draw_ms = drawDebugFrame(frame, result);
      draw_total_ms += result.draw_ms;
      if (++draw_frames >= 50) {
        std::clog << "[draw] 平均 " << draw_total_ms / draw_frames << "ms/帧（画图+imshow+waitKey）\n";
        draw_total_ms = 0.0;
        draw_frames = 0;
      }
    }

    // 作用：回调放在画图之后，和 P3 之前的顺序保持一致。
    if (callback) callback(frame, result);

    // 作用：把这一轮的三段耗时记下来，够 50 帧就打成一行。
    // 实现：read 里面含等帧，所以它大说明相机节流；process 大说明推理慢。
    if (config_.profile) {
      const auto write_done = std::chrono::steady_clock::now();
      const auto ms = [](const auto& a, const auto& b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
      };
      prof_read += ms(cycle_start, read_done);
      prof_process += ms(read_done, process_done);
      prof_write += ms(process_done, write_done);
      prof_cycle += ms(cycle_start, write_done);
      // P0：细分耗时直接从本帧结果拿，避免再埋一遍时钟。
      prof_acq += (read_done_time - result.frame_time) * 1000.0;
      prof_wait += result.attitude_wait_ms;
      prof_detect += result.detect_ms;
      prof_pnp += result.pnp_ms;
      prof_ekf += result.ekf_ms;
      prof_aim += result.aim_ms;
      if (++prof_frames >= 50) {
        const double n = static_cast<double>(prof_frames);
        std::clog << "[profile] cycle=" << prof_cycle / n << "ms  read=" << prof_read / n
                  << "ms  process=" << prof_process / n << "ms  write=" << prof_write / n << "ms\n";
        std::clog << "[profile]   acq=" << prof_acq / n << "ms  wait=" << prof_wait / n
                  << "ms  detect=" << prof_detect / n << "ms  pnp=" << prof_pnp / n
                  << "ms  ekf=" << prof_ekf / n << "ms  aim=" << prof_aim / n << "ms\n";
        prof_read = prof_process = prof_write = prof_cycle = 0.0;
        prof_acq = prof_wait = prof_detect = prof_pnp = prof_ekf = prof_aim = 0.0;
        prof_frames = 0;
      }
    }
  }

  // 收摊交给 run() 统一处理，这里只负责跑完。
}

// 作用：异步路径的等待端。实现：先起 perception 再起 capture，主线程只等停止。
void RealtimePipeline::runAsyncLoop(const std::atomic_bool& external_stop, FrameCallback callback) {
  frame_callback_ = std::move(callback);

  // perception 先起来，它会把 seen 对齐到当前槽序号，不会拿到上一轮残留值。
  perception_thread_ = std::jthread([this](std::stop_token token) { perceptionLoop(token); });
  capture_thread_ = std::jthread([this](std::stop_token token) { captureLoop(token); });
  if (config_.debug_draw) {
    debug_thread_ = std::jthread([this](std::stop_token token) { debugLoop(token); });
  }

  while (running_.load() && !external_stop.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

// 作用：capture 线程。实现：只读帧 + 独占化，不碰检测和滤波。
void RealtimePipeline::captureLoop(std::stop_token stop_token) {
  core::TimestampedFrame frame;
  uint64_t seq = 0;

  while (running_.load() && !stop_token.stop_requested()) {
    std::string error;
    const auto status = camera_->read(frame, &error);
    const double read_done_time = steadyNowSeconds();

    // 暂时没图不算错误，下一轮再要。
    if (status == core::FrameReadStatus::Timeout) continue;

    // 真读坏了才计丢帧并留错误，睡 5ms 防止忙等刷屏。
    if (status != core::FrameReadStatus::Ok) {
      {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.dropped_frames;
      }
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = std::move(error);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    // 相机内部的 Mat 会被下一帧复用，进槽前必须变成独占 buffer。
    auto packet = std::make_shared<FramePacket>();
    packet->frame = frame;
    packet->frame.bgr = frame.bgr.clone();
    packet->seq = ++seq;
    packet->read_done_time = read_done_time;
    packet->ready_time = steadyNowSeconds();

    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.captured_frames;
    }
    frame_slot_.publish(std::move(packet));
  }
}

// 作用：perception 线程。实现：取最新帧 -> processFrame -> CSV -> 发布调试帧 -> 回调。
void RealtimePipeline::perceptionLoop(std::stop_token stop_token) {
  // 以当前槽序号为起点，只处理之后的新帧，避开上一轮残留值。
  uint64_t seen = frame_slot_.seq();
  uint64_t last_seq = 0;
  std::shared_ptr<const FramePacket> packet;

  double prof_queue = 0.0;
  double prof_detect = 0.0;
  double prof_pnp = 0.0;
  double prof_ekf = 0.0;
  double prof_latency = 0.0;
  int prof_frames = 0;

  while (running_.load() && !stop_token.stop_requested()) {
    if (!frame_slot_.waitNewer(seen, packet, stop_token)) break;

    // 槽是覆盖写，序号跳了说明中间有帧没来得及处理。
    if (packet->seq > last_seq + 1) {
      const uint64_t skipped = packet->seq - last_seq - 1;
      std::lock_guard<std::mutex> lock(stats_mutex_);
      stats_.perception_skipped += skipped;
    }
    last_seq = packet->seq;

    const double process_start = steadyNowSeconds();
    PipelineFrameResult result = processFrame(packet->frame, packet->read_done_time);
    result.queue_wait_ms = (process_start - packet->ready_time) * 1000.0;

    // CSV 留在 perception，保证每个处理过的帧都有一行，方便和同步路径逐帧对比。
    writeCsv(result);
    if (config_.debug_draw) {
      debug_slot_.publish(DebugFrame{packet, result});
    }
    if (frame_callback_) frame_callback_(packet->frame, result);

    if (config_.profile) {
      prof_queue += result.queue_wait_ms;
      prof_detect += result.detect_ms;
      prof_pnp += result.pnp_ms;
      prof_ekf += result.ekf_ms;
      prof_latency += result.process_latency * 1000.0;
      if (++prof_frames >= 50) {
        const double n = static_cast<double>(prof_frames);
        std::clog << "[async] queue=" << prof_queue / n << "ms  detect=" << prof_detect / n
                  << "ms  pnp=" << prof_pnp / n << "ms  ekf=" << prof_ekf / n
                  << "ms  latency=" << prof_latency / n << "ms\n";
        prof_queue = prof_detect = prof_pnp = prof_ekf = prof_latency = 0.0;
        prof_frames = 0;
      }
    }
  }
}

// 作用：调试线程。实现：只画图和 imshow，画不过来就覆盖，不反压 perception。
void RealtimePipeline::debugLoop(std::stop_token stop_token) {
  uint64_t seen = debug_slot_.seq();
  DebugFrame debug;
  double total_ms = 0.0;
  int frames = 0;

  while (running_.load() && !stop_token.stop_requested()) {
    if (!debug_slot_.waitNewer(seen, debug, stop_token)) break;
    const double draw_ms = drawDebugFrame(debug.frame->frame, debug.result);
    total_ms += draw_ms;
    if (++frames >= 50) {
      std::clog << "[draw] 平均 " << total_ms / frames << "ms/帧（画图+imshow+waitKey）\n";
      total_ms = 0.0;
      frames = 0;
    }
  }

  // GUI 的 imshow 在本线程，收尾也留在这里，避免跨线程 destroyWindow。
  cv::destroyWindow("debug draw");
}

// 作用：画检测框、四面重投影和状态文字。实现：在副本上画，返回耗时毫秒。
double RealtimePipeline::drawDebugFrame(const core::TimestampedFrame& frame, const PipelineFrameResult& result) const {
  const auto draw_start = std::chrono::steady_clock::now();

  cv::Mat debug_frame = frame.bgr.clone();
  draw::drawDetections(debug_frame, result.detections);
  if (result.state_valid) {
    draw::drawFourArmorReprojections(debug_frame, result.state, result.R_C0C, processor_.cameraMatrix(),
                                     processor_.distortionCoeffs(), false);
  }
  draw::drawStateText(debug_frame, result.state);
  cv::imshow("debug draw", debug_frame);
  cv::waitKey(1);

  return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - draw_start).count();
}

// 作用：写实时 CSV。实现：只有同步循环或 perception 线程会调用，所以不加锁。
void RealtimePipeline::writeCsv(const PipelineFrameResult& result) {
  if (realtime_csv_) {
    realtime_csv_ << result.frame_time << ',' << (result.state_valid ? 1 : 0) << ',';
    for (int i = 0; i < 11; ++i) {
      realtime_csv_ << result.state.state(i) << ',';
    }
    realtime_csv_ << result.process_latency << ',' << result.valid_detections << ','
                  << (result.feedback_available ? 1 : 0) << "\r\n";
  }

  if (observation_csv_) {
    for (const auto& record : result.observations) {
      observation_csv_ << result.frame_time << ',' << record.azi << ',' << record.ele << ',' << record.dist << ','
                       << record.yaw << ',' << record.reproj_err << ',' << (record.sent_to_ekf ? 1 : 0) << "\r\n";
    }
  }
}

// 作用：打开 CSV 并写表头。实现：每次 start 重新建目录、截断文件；打不开就静默跳过。
void RealtimePipeline::openCsv() {
  std::error_code error;
  std::filesystem::create_directories(config_.output_directory, error);
  if (error) return;

  realtime_csv_.open(config_.output_directory + "/realtime.csv");
  if (realtime_csv_) {
    // 时间戳是 steady_clock 的秒数，量级 1e5，默认 6 位有效数字只剩 1 秒精度。
    realtime_csv_ << std::fixed << std::setprecision(6);
    realtime_csv_ << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r,dr,dz," << "latency_ms,detections,feedback\r\n";
  }

  if (config_.log_observations) {
    observation_csv_.open(config_.output_directory + "/observations.csv");
    if (observation_csv_) {
      observation_csv_ << std::fixed << std::setprecision(9);
      observation_csv_ << "time,azi,ele,dist,yaw,reproj_err,sent_to_ekf\r\n";
    }
  }
}

// 作用：关闭 CSV。实现：stop 在所有写者 join 之后调用。
void RealtimePipeline::closeCsv() {
  if (realtime_csv_.is_open()) realtime_csv_.close();
  if (observation_csv_.is_open()) observation_csv_.close();
}

// 作用：不带停止标志的 run 便利重载。实现：造一个永远为假的 atomic，转调带标志的版本。
void RealtimePipeline::run(FrameCallback callback) {
  std::atomic_bool never_stop{false};
  run(never_stop, std::move(callback));
}

// 作用：把整条链路安全关掉，可重复调用。实现：先停生产者再停消费者，最后关 CSV 和设备。
void RealtimePipeline::stop() {
  running_.store(false);

  // 顺序：capture 停掉后不会再有新帧；perception 把手上这帧做完就退；
  // debug 退出前会自己销毁窗口；feedback 最后停，保证前面处理时 IMU 还在。
  if (capture_thread_.joinable()) {
    capture_thread_.request_stop();
    capture_thread_.join();
  }
  if (perception_thread_.joinable()) {
    perception_thread_.request_stop();
    perception_thread_.join();
  }
  if (debug_thread_.joinable()) {
    debug_thread_.request_stop();
    debug_thread_.join();
  }
  if (feedback_thread_.joinable()) {
    feedback_thread_.request_stop();
    feedback_thread_.join();
  }

  // 所有 CSV 写者都已退出，现在关文件不会丢最后几行。
  closeCsv();

  // 作用：有相机就关，串口也顺手关掉。实现：空指针检查和 close。
  if (camera_) camera_->close();
  serial_.close();

  // 同步路径的 imshow 在主线程，窗口也在这里收；异步路径由 debug 线程负责。
  if (config_.debug_draw && !config_.pipeline.async) {
    cv::destroyWindow("debug draw");
  }
}

// 作用：给上层留的发送口，目前控制层还没用上。实现：串口 send，失败就统一记错误。
bool RealtimePipeline::send(const comm::VisionToGimbal& packet, std::string* error) {
  // 作用：发不出去得让人知道。实现：send 返回假就调 fail 打包错误。
  if (!serial_.send(packet)) {
    return fail("串口发送失败", error);
  }
  return true;
}

// 作用：线程安全地取最新滤波状态。实现：锁内拷贝一份，再返回 valid。
bool RealtimePipeline::latestState(kalfilter::KFResult& state) const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  state = latest_state_;
  return state.valid;
}

// 作用：取延迟统计快照，内部已经自带锁。实现：转调 latency_.snapshot()。
kalfilter::LatencySnapshot RealtimePipeline::latencySnapshot() const { return latency_.snapshot(); }

// 作用：取计数器快照。实现：锁内整体拷贝 stats_，避免读到改了一半。
PipelineStats RealtimePipeline::stats() const {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  return stats_;
}

// 作用：把一帧从像素加工到滤波状态，是整条链路的正餐。
// 实现：等姿态、算相对旋转、跑处理器、发布状态、记延迟；画图和回调由调用方负责。
PipelineFrameResult RealtimePipeline::processFrame(const core::TimestampedFrame& frame, double frame_received_time) {
  // 作用：先备好结果容器，填上帧时间。实现：栈上默认构造后赋 timestamp。
  PipelineFrameResult result;
  result.frame_time = frame.timestamp;

  // 作用：处理数先加一，成败都算处理过。实现：锁内自增 processed_frames。
  {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.processed_frames;
  }

  // 作用：先跑检测，检测不依赖姿态。实现：单独调 processor_.detect。
  // 这么排是因为推理要几毫秒，这段时间正好够 IMU 包落地，后面拿姿态就不用干等。
  const auto detections = processor_.detect(frame);

  // 作用：去历史里找这一帧时刻的云台姿态。实现：调 waitForAttitude，带超时等待。
  kalfilter::GimbalSample attitude;
  // P0：这一段是纯等待，单列出来才知道 IMU 链路到底贡献了多少延迟。
  const auto attitude_wait_start = std::chrono::steady_clock::now();
  // 作用：无 IMU 时用单位四元数顶替，世界系就等于初始相机系。实现：imu_enabled 为假才走这一支。
  if (!config_.imu_enabled) {
    attitude.host_time = frame.timestamp;
    attitude.board_time_us = 0;
    attitude.q.setIdentity();
  }
  // 作用：等不到就丢帧，别拿旧姿态凑合。实现：返回假就计数 dropped_frames 并原样返回空结果。
  else if (!waitForAttitude(frame.timestamp, attitude)) {
    {
      std::lock_guard<std::mutex> lock(stats_mutex_);
      ++stats_.dropped_frames;
    }
    return result;
  } else {
    // 作用：姿态到手，标记本帧反馈可用。实现：置 true。
    result.feedback_available = true;
  }
  result.attitude_wait_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - attitude_wait_start).count();

  // 作用：第一次成功拿到姿态就把它当世界系基准。实现：没初始化过就存 q_init_ 并置标志。
  // 注意：有 IMU 时世界是水平系，q_init_ 只保留给无 IMU 路径和瞄准兜底。
  if (!has_q_init_) {
    q_init_ = attitude.q;
    has_q_init_ = true;
  }

  // 作用：算相机系到算法世界的旋转。实现：有 IMU 时用 C 板 pitch/yaw 把相机系直接转到
  // 重力水平系；没有 IMU 时退回相对首帧相机系（世界跟着相机一起歪）。
  Eigen::Matrix3d R_C0C;
  if (config_.imu_enabled) {
    constexpr double kDeg2Rad = 0.017453292519943295;
    const double pitch =
        attitude.pitch * config_.level_pitch_sign + config_.level_pitch_offset_deg * kDeg2Rad;
    R_C0C = cameraToLevelWorld(pitch, attitude.yaw);
    if (!has_level_log_) {
      has_level_log_ = true;
      const Eigen::Vector3d q_axis = attitude.q * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d py_axis(std::cos(pitch) * std::cos(attitude.yaw), std::cos(pitch) * std::sin(attitude.yaw),
                                    std::sin(pitch));
      std::clog << std::fixed << std::setprecision(2) << "[level] 板 yaw=" << attitude.yaw * kRad2Deg
                << "° pitch=" << attitude.pitch * kRad2Deg << "° (sign=" << config_.level_pitch_sign
                << ", offset=" << config_.level_pitch_offset_deg << "°)"
                << " | q+x az=" << std::atan2(q_axis.y(), q_axis.x()) * kRad2Deg
                << "° el=" << std::asin(std::clamp(q_axis.z(), -1.0, 1.0)) * kRad2Deg << "°"
                << " | pitch/yaw 光轴=(" << py_axis.x() << "," << py_axis.y() << "," << py_axis.z() << ")\n";
    }
  } else {
    // 作用：算从初始姿态到当前姿态的相对旋转，算法世界 C0 就建立在这上面。实现：共轭乘当前姿态再单位化。
    const Eigen::Quaterniond q_relative = (q_init_.conjugate() * attitude.q).normalized();
    // 作用：把四元数变成旋转矩阵，PnP 结果转世界系要用。实现：toRotationMatrix。
    R_C0C = q_relative.toRotationMatrix();
  }

  // 作用：把帧、检测结果和旋转交给下游处理器，走细化、PnP、EKF。实现：一次 solve 调用全套。
  const FrameProcessingResult processing = processor_.solve(frame, detections, R_C0C);
  // 作用：把有效检测数、滤波状态、可信标志搬进对外结果。实现：逐字段赋值，检测数取 observations 大小。
  result.valid_detections = static_cast<int>(processing.observations.size());
  // 调试线程要重新画检测框和四面重投影，这里把素材带出去。
  result.detections = processing.detections;
  result.R_C0C = R_C0C;
  // P0：把分段耗时透传给外部，profile 和延迟统计都靠它。
  result.detect_ms = processing.detect_timings.preprocess_ms + processing.detect_timings.inference_ms +
                     processing.detect_timings.postprocess_ms;
  result.pnp_ms = processing.pnp_ms;
  result.ekf_ms = processing.ekf_ms;

  // 作用：把原始观测原样留一份给外部落盘，标定 R 靠它。实现：逐条拷成轻量记录。
  for (const auto& observation : processing.observations) {
    ObservationRecord record;
    record.azi = observation.ypd.azi;
    record.ele = observation.ypd.ele;
    record.dist = observation.ypd.dist;
    record.yaw = observation.ekf_yaw;
    record.reproj_err = observation.pose.err;
    record.sent_to_ekf = observation.sent_to_ekf;
    result.observations.push_back(record);
  }
  result.state = processing.state;
  result.state_valid = processing.state_valid;

  // 作用：好状态就发布出去，供 UI 或控制层取用。实现：valid 时锁内覆盖 latest_state_。
  if (result.state_valid) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_state_ = result.state;
  }

  // 作用：统计有效帧数，只在滤波器活着且状态可信时加。实现：先看 ekf_initialized，再看 state_valid，锁内自增。
  if (processing.ekf_initialized) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    if (result.state_valid) ++stats_.valid_frames;
  }

  // 作用：记下命令可发的时刻，算出本帧总处理延迟。实现：取当前 steady 时间减帧时间戳。
  const double command_ready_time = steadyNowSeconds();
  result.process_latency = command_ready_time - frame.timestamp;

  // 作用：凑一份打点交给延迟统计，目前只填了首尾和到达时间。实现：构造 FrameTiming 后调 recordFrame。
  kalfilter::FrameTiming timing;
  timing.exposure_time = frame.timestamp;
  // P0：收帧时刻用 read() 返回的真实时刻，之前填的是曝光时刻，采集段等于被抹平了。
  timing.frame_received_time = frame_received_time;
  timing.detection_done_time = processing.detect_done_time;
  timing.pnp_done_time = processing.pnp_done_time;
  timing.ekf_done_time = processing.ekf_done_time;
  timing.command_ready_time = command_ready_time;
  latency_.recordFrame(timing);

  // 作用：状态可信就解一条云台指令。实现：算不发也行，debug_log 打开时会打印出来给实测判断。
  if (config_.control.send || config_.control.debug_log) {
    // P0：解算 + 串口写单独计时，串口阻塞会直接反映在这里。
    const auto aim_start = std::chrono::steady_clock::now();
    issueCommand(frame, attitude, result.state, command_ready_time, result.valid_detections);
    result.aim_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - aim_start).count();
  }

  // 调试画图和回调都由调用方在算法之外处理：同步路径在当前线程，异步路径在 debug 线程。
  return result;
}

// 作用：把滤波状态解成一条云台指令，按配置打印或下发。实现：AimSolver 出角度，填包后交给 send()。
void RealtimePipeline::issueCommand(const core::TimestampedFrame& frame, const kalfilter::GimbalSample& attitude,
                                    const kalfilter::KFResult& state, double command_ready_time, int detections) {
  // 作用：预测到指令生效时刻，超前量 = 本帧已花的处理时间 + 通信延迟，飞行时间在解算里迭代出来。
  const double latency = (command_ready_time - frame.timestamp) + config_.serial.communication_latency;

  // 有 IMU 时 EKF 状态在水平世界 E 里，瞄准点直接乘 E→C 板世界系，不再走 q_init/q_now。
  const Eigen::Matrix3d R_E_to_board = levelToBoardWorld();

  const control::AimSolution solution =
      aim_.solve(state, q_init_, attitude.q, latency, attitude.bullet_speed, command_ready_time,
                 config_.imu_enabled ? &R_E_to_board : nullptr);

  // 作用：限流打印，0.5 s 一行。实现：第一次无条件打，之后隔 0.5 s。
  const bool log_due = config_.control.debug_log &&
                       (last_aim_log_time_ < 0.0 || command_ready_time - last_aim_log_time_ >= 0.5);
  if (log_due) last_aim_log_time_ = command_ready_time;

  if (!solution.valid) {
    // 状态还没起来也要把 C 板自报的姿态打出来，测坐标系用不着等 EKF 收敛。
    if (log_due) {
      std::clog << std::fixed << std::setprecision(2) << "[aim] 状态无效 dets=" << detections << " | ";
      logAttitudeConvention(attitude);
      std::clog << '\n';
    }
    return;
  }

  // 作用：按协议填包。角度单位是弧度，和 C 板反馈一致。实现：逐字段赋值，vel/acc 由差分得到。
  comm::VisionToGimbal packet{};
  // C 板时钟上的"这条指令生成时刻"：本帧姿态的板卡时间 + 上位机侧流过的时间。
  // 单位跟 C 板自己发来的 DWT_stamp 一致（毫秒）；board_time_us 是展开后的微秒，先换回毫秒。
  const double age_seconds = std::max(0.0, command_ready_time - attitude.host_time);
  packet.time_stamp =
      attitude.board_time_us / 1000 + static_cast<uint64_t>(age_seconds * 1000.0);
  packet.mode = config_.control.mode;
  packet.pitch = static_cast<float>(solution.pitch);
  packet.pitch_vel = static_cast<float>(solution.pitch_vel);
  packet.pitch_acc = static_cast<float>(solution.pitch_acc);
  packet.yaw = static_cast<float>(solution.yaw);
  packet.yaw_vel = static_cast<float>(solution.yaw_vel);
  packet.yaw_acc = static_cast<float>(solution.yaw_acc);

  if (log_due) {
    std::clog << std::fixed << std::setprecision(2) << "[aim] cmd yaw=" << solution.yaw * kRad2Deg
              << "° pitch=" << solution.pitch * kRad2Deg << "° id=" << solution.armor_id
              << " dist=" << solution.distance << "m fly=" << solution.flight_time * 1e3
              << "ms stamp=" << packet.time_stamp << " age=" << age_seconds * 1e3 << "ms dets=" << detections
              << " | ";
    logAttitudeConvention(attitude);
    std::clog << '\n';
  }

  if (!config_.control.send) return;

  std::string error;
  if (!send(packet, &error)) {
    if (!send_error_reported_) {
      send_error_reported_ = true;
      std::cerr << "[aim] 指令发送失败: " << error << "（后续同类失败不再重复打印）\n";
    }
  } else {
    send_error_reported_ = false;
  has_feedback_log_.store(false);
  }
}

// 作用：等云台历史覆盖到目标时刻，最多等配置的毫秒数。实现：循环尝试插值，没成且没停就睡 1ms，到点或停机即认输。
bool RealtimePipeline::waitForAttitude(double host_time, kalfilter::GimbalSample& sample) {
  // 作用：算出等待截止时刻，等待时长从配置来，负数按 0 处理。实现：now 加上转换成 steady duration 的毫秒数。
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                            std::chrono::duration<double, std::milli>(std::max(0.0, config_.timing.attitude_wait_ms)));

  // 作用：反复尝试，直到拿到姿态、停机或超时。实现：无限循环里三个出口。
  for (;;) {
    // 作用：历史已经覆盖目标时刻，直接成功。实现：interpolate 返回真就 return true。
    if (history_.interpolate(host_time, sample)) return true;
    // 作用：链路都停了就别等了。实现：running_ 为假返回 false。
    if (!running_.load()) return false;
    // 作用：等到头还没来，就别再堵了。实现：超期直接跳出循环去外推。
    if (std::chrono::steady_clock::now() >= deadline) break;
    // 作用：别死循环烧 CPU，小睡 1ms 再问。实现：sleep_for 一毫秒。
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // 作用：包还没到就用角速度把姿态推到该帧时刻，最多推 max_prediction_seconds。
  // 实现：history_.predict 内部限制外推距离，推不了就返回假。
  if (history_.predict(host_time, sample)) {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.extrapolated_attitude;
    return true;
  }

  return false;
}

// 作用：后台线程专收 C 板反馈，收一条处理一条。实现：poll 循环把串口缓冲抽干，没收到包就睡 1ms，直到停机。
void RealtimePipeline::feedbackLoop(std::stop_token stop_token) {
  // 作用：复用同一个包对象，省得每轮构造。实现：循环外声明。
  comm::GimbalToVision packet;

  // 作用：起跑时刻记下来，2 秒还没收到合法包就提醒一次。实现：循环外取一次 steady 时间。
  const double loop_start = steadyNowSeconds();
  bool warned_no_feedback = false;

  // 作用：只要还在跑且没被请求停止就继续收。实现：while 同时检查 running_ 和 stop_token。
  while (running_.load() && !stop_token.stop_requested()) {
    if (!has_feedback_log_.load() && !warned_no_feedback && steadyNowSeconds() - loop_start > 2.0) {
      warned_no_feedback = true;
      std::cerr << "[C板] 串口开着但两秒内没收到合法包：检查波特率、协议版本和校验和。\n";
    }
    // 作用：记录这一轮有没有收到东西，决定要不要睡。实现：bool 标志。
    bool received = false;
    // 作用：把串口里积压的包一口气全取出来。实现：while poll 成功就处理，直到没包可读。
    while (serial_.poll(packet)) {
      received = true;
      handleFeedback(packet);
    }

    // 作用：空转时让出 CPU，1ms 后再说。实现：没收到包就 sleep_for。
    if (!received) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}

// 作用：把一条原始反馈包翻译成云台样本并入历史。实现：先做时钟同步，再逐字段拷贝，最后 push 成功才计数。
void RealtimePipeline::handleFeedback(const comm::GimbalToVision& packet) {
  // 作用：记下包到达本机的时刻，时钟同步要用它倒推事件时间。实现：取当前 steady 秒数。
  const double receive_time = steadyNowSeconds();

  // 作用：准备接收同步结果：展开后的板卡微秒和对应的上位机时刻。实现：两个局部变量，由 observeAndMap 填。
  uint64_t board_time_us{};
  double event_host_time{};
  // 作用：同步失败说明戳有问题（乱序、越界），这条反馈不要也罢。实现：DWT_stamp 是毫秒，ClockSync 按 ms 展开再换成 μs。
  if (!clock_.observeAndMap(packet.DWT_stamp, receive_time, board_time_us, event_host_time)) {
    return;
  }

  // 作用：把包里的字段拼成历史样本。实现：先建对象，再逐字段赋值，四元数按 wxyz 顺序构造。
  kalfilter::GimbalSample sample;
  sample.host_time = event_host_time;
  sample.board_time_us = board_time_us;
  sample.q = Eigen::Quaterniond(packet.q[0], packet.q[1], packet.q[2], packet.q[3]);
  sample.pitch = packet.pitch;
  sample.pitch_vel = packet.pitch_vel;
  sample.yaw = packet.yaw;
  sample.yaw_vel = packet.yaw_vel;
  sample.bullet_speed = packet.bullet_speed;
  sample.mode = packet.mode;

  // 作用：入库失败一般是时间没前进，直接丢弃，不入账。实现：push 返回假就 return。
  if (!history_.push(sample)) return;

  // 作用：第一条合法反馈打一行，确认协议解析、时钟同步和字段都对。实现：只打一次。
  if (!has_feedback_log_.load()) {
    has_feedback_log_.store(true);
    std::clog << std::fixed << std::setprecision(2) << "[C板] 首条反馈 DWT=" << packet.DWT_stamp
              << "ms -> 上位机 " << event_host_time << "s, pitch=" << packet.pitch * kRad2Deg
              << "° yaw=" << packet.yaw * kRad2Deg << "°, 弹速=" << packet.bullet_speed
              << ", mode=" << static_cast<int>(packet.mode) << '\n';
  }

  // 作用：更新“最近收到反馈”的时刻，顺便累计收包数。实现：atomic store 加锁内自增。
  last_feedback_receive_time_.store(receive_time);
  std::lock_guard<std::mutex> lock(stats_mutex_);
  ++stats_.received_feedback;
}

// 作用：统一处理失败路径，记录并上报错误。实现：写 last_error_，外部指针非空就一并拷过去。
bool RealtimePipeline::fail(const std::string& message, std::string* error) {
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
  }
  if (error) *error = message;
  return false;
}

}  // namespace realtime
