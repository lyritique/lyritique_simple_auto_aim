# ifndef AUTOAIM_REALTIME_PIPELINE_HPP
# define AUTOAIM_REALTIME_PIPELINE_HPP

# include <atomic>
# include <cstdint>
# include <functional>
# include <memory>
# include <mutex>
# include <string>
# include <thread>

# include "core/config.hpp"
# include "core/frame.hpp"
# include "runtime/frame_processor.hpp"
# include "timing/clock_sync.hpp"
# include "timing/gimbal_history.hpp"
# include "timing/latency_estimator.hpp"
# include "sensors/camera_source.hpp"
# include "transport/protocol.hpp"
# include "transport/serial_link.hpp"

namespace realtime
{

using RealtimePipelineConfig = core::AppConfig;

struct PipelineFrameResult
{
    double frame_time {};
    double process_latency {};

    int valid_detections {};
    bool feedback_available {};
    bool state_valid {};

    kalfilter::KFResult state;
};

struct PipelineStats
{
    uint64_t received_feedback {};
    uint64_t processed_frames {};
    uint64_t dropped_frames {};
    uint64_t valid_frames {};
};

using FrameCallback = std::function<void(
    const core::TimestampedFrame&,
    const PipelineFrameResult&)>;

/*
 * 实时链路：
 * C 板反馈线程 -> ClockSync -> GimbalFeedbackHistory
 * MVS 相机帧 -> 按曝光/到达时间插值姿态 -> PnP -> EKF
 *
 * 当前版本不自动发送控制指令；send() 留给后续弹道和控制层。
 */
class RealtimePipeline
{
public:
    explicit RealtimePipeline(RealtimePipelineConfig config = {});
    ~RealtimePipeline();

    RealtimePipeline(const RealtimePipeline&) = delete;
    RealtimePipeline& operator=(const RealtimePipeline&) = delete;

    bool start(std::string* error = nullptr);
    void run(
        const std::atomic_bool& external_stop,
        FrameCallback callback = {});
    void run(FrameCallback callback = {});
    void stop();

    bool send(
        const comm::VisionToGimbal& packet,
        std::string* error = nullptr);

    bool latestState(kalfilter::KFResult& state) const;
    kalfilter::LatencySnapshot latencySnapshot() const;
    PipelineStats stats() const;
    const RealtimePipelineConfig& config() const noexcept { return config_; }

private:
    PipelineFrameResult processFrame(
        const core::TimestampedFrame& frame,
        const FrameCallback& callback);
    bool waitForAttitude(
        double host_time,
        kalfilter::GimbalSample& sample);
    void feedbackLoop(std::stop_token stop_token);
    void handleFeedback(const comm::GimbalToVision& packet);
    bool fail(const std::string& message, std::string* error);

    RealtimePipelineConfig config_;

    std::unique_ptr<camera::CameraSource> camera_;
    comm::SerialLink serial_;
    kalfilter::ClockSync clock_;
    kalfilter::GimbalFeedbackHistory history_;
    kalfilter::LatencyEstimator latency_;

    FrameProcessor processor_;

    std::atomic_bool running_ {false};
    std::jthread feedback_thread_;

    mutable std::mutex state_mutex_;
    kalfilter::KFResult latest_state_;

    mutable std::mutex stats_mutex_;
    PipelineStats stats_;

    std::atomic<double> last_feedback_receive_time_ {0.0};

    bool has_q_init_ {};
    Eigen::Quaterniond q_init_ {Eigen::Quaterniond::Identity()};

    std::string last_error_;
};

}   // namespace realtime

# endif
