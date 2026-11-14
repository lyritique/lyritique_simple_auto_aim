#ifndef AUTOAIM_REALTIME_PIPELINE_HPP
#define AUTOAIM_REALTIME_PIPELINE_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/config.hpp"
#include "core/frame.hpp"
#include "runtime/frame_processor.hpp"
#include "sensors/camera_source.hpp"
#include "timing/clock_sync.hpp"
#include "timing/gimbal_history.hpp"
#include "timing/latency_estimator.hpp"
#include "transport/protocol.hpp"
#include "transport/serial_link.hpp"

// 成员与线程职责、start 的资源回滚、读帧循环、姿态等待、反馈线程、时钟同步入库

namespace realtime {

using RealtimePipelineConfig = core::AppConfig;

// 作用：一条原始观测记录，只用于标定观测噪声，不参与滤波。实现：YPD 加朝向角。
struct ObservationRecord {
  double azi{};
  double ele{};
  double dist{};
  double yaw{};
  double reproj_err{};
  bool sent_to_ekf{};
};

struct PipelineFrameResult {
  // 作用：这一帧的时间戳，下游对齐和日志都靠它。实现：从 frame.timestamp 拷进来。
  double frame_time{};
  // 作用：从曝光到命令可发的耗时，衡量实时性。实现：命令就绪时刻减帧时间戳。
  double process_latency{};

  // 作用：本帧过了 PnP 的板子数量。实现：取 observations 的 size。
  int valid_detections{};
  // 作用：本帧有没有拿到可用云台反馈。实现：姿态插值成功才置 true。
  bool feedback_available{};
  // 作用：EKF 状态能不能信。实现：透传 processing.state_valid。
  bool state_valid{};

  // 作用：EKF 的最终状态整包。实现：直接拷贝 processing.state。
  kalfilter::KFResult state;

  // 作用：本帧所有原始观测，标定 R 时用。实现：从 FrameObservation 逐条拷成轻量记录。
  std::vector<ObservationRecord> observations;
};

struct PipelineStats {
  // 作用：累计收到多少条 C 板反馈。实现：handleFeedback 成功入库后自增。
  uint64_t received_feedback{};
  // 作用：累计处理了多少帧。实现：processFrame 进门先自增。
  uint64_t processed_frames{};
  // 作用：累计丢了多少帧。实现：读图失败或姿态等待超时都会加一。
  uint64_t dropped_frames{};
  // 作用：累计有多少帧产出了有效状态。实现：EKF 已初始化且 state.valid 才自增。
  uint64_t valid_frames{};
  // 作用：累计有多少帧的姿态是靠外推补的。实现：waitForAttitude 走 predict 分支时自增。
  uint64_t extrapolated_attitude{};
};

// 作用：每帧处理完的回调类型，方便外面接 UI、录包或发指令。实现：std::function 包住“原始帧 + 结果”两个参数。
using FrameCallback = std::function<void(const core::TimestampedFrame&, const PipelineFrameResult&)>;

/*
 * 实时链路：
 * C 板反馈线程 -> ClockSync -> GimbalFeedbackHistory
 * MVS 相机帧 -> 按曝光/到达时间插值姿态 -> PnP -> EKF
 *
 * 当前版本不自动发送控制指令；send() 留给后续弹道和控制层。
 */
class RealtimePipeline {
 public:
  // 作用：拿配置把整条链路装配好。实现：移动配置，顺手把 ClockSync、历史缓存、帧处理器都按参数构造。
  // 参数：config —— 全量配置，按值传入，内部 move 进成员；传左值就多一次拷贝（默认值给空配置方便测试）。
  // 返回：无。此时设备没开、线程没起，真正干活要等 start()。
  explicit RealtimePipeline(RealtimePipelineConfig config = {});
  // 作用：析构时自动收摊，别留半开着的相机和串口。实现：直接调 stop()。
  ~RealtimePipeline();

  // 作用：这条链路独占相机和串口，禁止复制。实现：拷贝构造和赋值都 delete。
  RealtimePipeline(const RealtimePipeline&) = delete;
  RealtimePipeline& operator=(const RealtimePipeline&) = delete;

  // 作用：清状态、开串口、开相机、起反馈线程，一条龙启动。实现：先 stop 再逐步初始化，任一步失败就回滚并报错。
  // 参数：error —— 可选出错信息出口，非空时失败原因会写进去。
  // 返回：true 表示设备全开、反馈线程已跑；false 表示没起来，相机和串口都已回滚关闭。
  bool start(std::string* error = nullptr);
  // 作用：主循环，读帧读到外部喊停。实现：轮询相机，超时就下一轮，出错计数，成功就丢给 processFrame。
  // 参数：external_stop —— 外部停止标志，置 true 后本轮循环退出；callback —— 每帧回调，可空。
  // 返回：无。正常退出时会顺手调 stop()，不用再手动收拾。
  void run(const std::atomic_bool& external_stop, FrameCallback callback = {});
  // 作用：不给停止标志的简化版 run，跑到天荒地老。实现：造一个永远为假的 atomic 再转调上面的重载。
  // 参数：callback —— 每帧回调，可空。返回：无，只能靠进程信号或对象销毁来结束。
  void run(FrameCallback callback = {});
  // 作用：关线程、关相机、关串口，安全退出。实现：清 running_，请求停止并 join 反馈线程，再依次关设备。
  // 参数：无。返回：无。可重复调用，没启动也不报错。
  void stop();

  // 作用：把控制包发回 C 板，当前版本先留着接口。实现：串口 send，失败就记错误。
  // 参数：packet —— 要下发的视觉指令；error —— 可选出错信息出口。
  // 返回：true 发送成功；false 串口没开或写失败，细节看 error/last_error_。
  bool send(const comm::VisionToGimbal& packet, std::string* error = nullptr);

  // 作用：拿最近一次有效状态，线程安全。实现：锁内拷贝 latest_state_，valid 一并返回。
  // 参数：state —— 输出参数，会被覆盖。返回：state.valid，false 表示还没产出过可信状态。
  bool latestState(kalfilter::KFResult& state) const;
  // 作用：拿延迟统计快照。实现：转调 latency_.snapshot()。
  // 参数：无。返回：三段延迟的统计值，valid 为假时 total_delay 无意义。
  kalfilter::LatencySnapshot latencySnapshot() const;
  // 作用：拿计数器统计。实现：锁内拷贝 stats_。
  // 参数：无。返回：四类累计计数的一份拷贝，随时可调用。
  PipelineStats stats() const;
  // 作用：只读配置，方便外部查参数。实现：内联返回常量引用，不拷贝。
  // 参数：无。返回：构造时存下的配置引用，跟随对象生命周期。
  const RealtimePipelineConfig& config() const noexcept { return config_; }

 private:
  // 作用：一帧的完整加工：等姿态、算相对旋转、跑处理、记延迟、回调。实现：内部按顺序调用各组件，失败就计数返回。
  // 参数：frame —— 已读出的帧；callback —— 外部回调，可空。返回：本帧结果，姿态等不到时各字段保持默认。
  PipelineFrameResult processFrame(const core::TimestampedFrame& frame, const FrameCallback& callback);
  // 作用：等云台历史覆盖到目标时刻，最多等配置的毫秒数。实现：循环尝试插值，睡 1ms 再试，超时或停机就放弃。
  // 参数：host_time —— 要查的时刻；sample —— 输出姿态。返回：true 拿到姿态，false 超时或链路已停。
  bool waitForAttitude(double host_time, kalfilter::GimbalSample& sample);
  // 作用：独立线程专收串口反馈。实现：poll 到包就处理，没包就睡 1ms，直到停止请求。
  // 参数：stop_token —— jthread 的停止令牌。返回：无，线程函数自己跑到停机条件成立。
  void feedbackLoop(std::stop_token stop_token);
  // 作用：把一条原始包变成可用的云台样本并入库。实现：时钟同步、字段拷贝、history_.push、计数。
  // 参数：packet —— 串口刚收上来的反馈包。返回：无，同步失败或时间不递增就静默丢弃。
  void handleFeedback(const comm::GimbalToVision& packet);
  // 作用：统一记错误并返回失败。实现：写 last_error_，有外部指针就一并带出。
  // 参数：message —— 错误原因；error —— 可选出错信息出口。返回：恒定 false，方便直接 return fail(...)。
  bool fail(const std::string& message, std::string* error);

  // 作用：整条链路的参数总仓。实现：构造时从外部移动进来，之后只读。
  RealtimePipelineConfig config_;

  // 作用：相机抽象，实时是 MVS，回放是视频文件。实现：unique_ptr 多态持有，start 时才 new。
  std::unique_ptr<camera::CameraSource> camera_;
  // 作用：串口链路，反馈和指令都走它。实现：值成员，open/close/send/poll 全由它管。
  comm::SerialLink serial_;
  // 作用：板卡时钟到上位机时钟的翻译官。实现：值成员，构造时吃通信延迟参数。
  kalfilter::ClockSync clock_;
  // 作用：按时间存云台姿态，供插值和短时预测。实现：值成员，构造时吃三个时长参数。
  kalfilter::GimbalFeedbackHistory history_;
  // 作用：统计处理、通信、飞行三段延迟。实现：值成员，内部自带滑窗和锁。
  kalfilter::LatencyEstimator latency_;

  // 作用：真正干检测和滤波的处理器。实现：值成员，构造时把 config_ 传进去。
  FrameProcessor processor_;

  // 作用：运行开关，主循环和反馈线程都盯着它。实现：atomic bool，start 置真、stop 置假。
  std::atomic_bool running_{false};
  // 作用：后台收包线程的把手。实现：std::jthread，start 时创建，stop 时请求停止并 join。
  std::jthread feedback_thread_;

  // 作用：保护最新状态，写滤波线程、读外部线程。实现：mutable mutex，读写都先 lock。
  mutable std::mutex state_mutex_;
  // 作用：最近一次有效滤波结果，给 UI 或控制层兜底。实现：KFResult 值成员，只在 valid 时覆盖。
  kalfilter::KFResult latest_state_;

  // 作用：保护计数器，多个线程都在加。实现：mutable mutex 加普通结构体。
  mutable std::mutex stats_mutex_;
  // 作用：四类累计计数。实现：PipelineStats 值成员，改之前先拿锁。
  PipelineStats stats_;

  // 作用：最近一次收到反馈的时刻，可用来判断链路断没断。实现：atomic double，存 steady_clock 秒数。
  std::atomic<double> last_feedback_receive_time_{0.0};

  // 作用：记住第一帧的姿态有没有存过，决定 q_init_ 要不要初始化。实现：bool 门禁。
  bool has_q_init_{};
  // 作用：作为世界系基准的初始姿态，之后都用它算相对旋转。实现：Eigen 四元数，首帧赋值。
  Eigen::Quaterniond q_init_{Eigen::Quaterniond::Identity()};

  // 作用：留下最后一条错误信息，方便事后查账。实现：std::string，fail() 里覆盖。
  std::string last_error_;
};

}  // namespace realtime

#endif
