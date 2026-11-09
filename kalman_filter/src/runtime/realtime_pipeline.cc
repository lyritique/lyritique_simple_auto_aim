# include "runtime/realtime_pipeline.hpp"

# include "sensors/mvs_camera.hpp"

# include <algorithm>
# include <chrono>
# include <utility>

namespace realtime
{
namespace
{

double steadyNowSeconds()
{
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

}   // namespace

RealtimePipeline::RealtimePipeline(RealtimePipelineConfig config)
    : config_(std::move(config)),
      clock_(config_.serial.communication_latency),
      history_(
          config_.timing.max_history_seconds,
          config_.timing.max_attitude_gap_seconds,
          config_.timing.max_attitude_prediction_seconds),
      processor_(config_)
{
}

RealtimePipeline::~RealtimePipeline()
{
    stop();
}

bool RealtimePipeline::start(std::string* error)
{
    stop();
    last_error_.clear();

    clock_.reset();
    clock_.setCommLatency(config_.serial.communication_latency);
    history_.clear();
    latency_.clear();
    latency_.recordCommunication(
        config_.serial.communication_latency);

    processor_.reset();
    has_q_init_ = false;
    q_init_.setIdentity();
    last_feedback_receive_time_.store(0.0);

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = PipelineStats {};
    }

    if (config_.serial.device.empty())
    {
        return fail("串口设备路径为空", error);
    }

    if (!serial_.open(
            config_.serial.device.c_str(),
            config_.serial.baud))
    {
        return fail(
            "串口打开失败: " + config_.serial.device +
                " @" + std::to_string(config_.serial.baud),
            error);
    }

    camera_ = std::make_unique<camera::MvsCamera>(config_.camera);
    if (!camera_->open(error))
    {
        serial_.close();
        camera_->close();
        return false;
    }

    running_.store(true);
    feedback_thread_ = std::jthread(
        [this](std::stop_token token)
        {
            feedbackLoop(token);
        });
    return true;
}

void RealtimePipeline::run(
    const std::atomic_bool& external_stop,
    FrameCallback callback)
{
    if (!running_.load()) return;

    while (running_.load() && !external_stop.load())
    {
        core::TimestampedFrame frame;
        std::string error;
        const auto status = camera_->read(frame, &error);
        if (status == core::FrameReadStatus::Timeout)
        {
            continue;
        }
        if (status != core::FrameReadStatus::Ok)
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.dropped_frames;
            last_error_ = std::move(error);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5));
            continue;
        }

        processFrame(frame, callback);
    }

    stop();
}

void RealtimePipeline::run(FrameCallback callback)
{
    std::atomic_bool never_stop {false};
    run(never_stop, std::move(callback));
}

void RealtimePipeline::stop()
{
    running_.store(false);

    if (feedback_thread_.joinable())
    {
        feedback_thread_.request_stop();
        feedback_thread_.join();
    }

    if (camera_) camera_->close();
    serial_.close();
}

bool RealtimePipeline::send(
    const comm::VisionToGimbal& packet,
    std::string* error)
{
    if (!serial_.send(packet))
    {
        return fail("串口发送失败", error);
    }
    return true;
}

bool RealtimePipeline::latestState(kalfilter::KFResult& state) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    state = latest_state_;
    return state.valid;
}

kalfilter::LatencySnapshot RealtimePipeline::latencySnapshot() const
{
    return latency_.snapshot();
}

PipelineStats RealtimePipeline::stats() const
{
    std::lock_guard<std::mutex> lock(stats_mutex_);
    return stats_;
}

PipelineFrameResult RealtimePipeline::processFrame(
    const core::TimestampedFrame& frame,
    const FrameCallback& callback)
{
    PipelineFrameResult result;
    result.frame_time = frame.timestamp;

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.processed_frames;
    }

    kalfilter::GimbalSample attitude;
    if (!waitForAttitude(frame.timestamp, attitude))
    {
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.dropped_frames;
        }
        return result;
    }

    result.feedback_available = true;

    if (!has_q_init_)
    {
        q_init_ = attitude.q;
        has_q_init_ = true;
    }

    const Eigen::Quaterniond q_relative =
        (q_init_.conjugate() * attitude.q).normalized();
    const Eigen::Matrix3d R_C0C =
        q_relative.toRotationMatrix();

    const FrameProcessingResult processing =
        processor_.process(frame, R_C0C);
    result.valid_detections = static_cast<int>(
        processing.observations.size());
    result.state = processing.state;
    result.state_valid = processing.state_valid;

    if (result.state_valid)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_state_ = result.state;
    }

    if (processing.ekf_initialized)
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        if (result.state_valid) ++stats_.valid_frames;
    }

    const double command_ready_time = steadyNowSeconds();
    result.process_latency =
        command_ready_time - frame.timestamp;

    kalfilter::FrameTiming timing;
    timing.exposure_time = frame.timestamp;
    timing.frame_received_time = frame.timestamp;
    timing.command_ready_time = command_ready_time;
    latency_.recordFrame(timing);

    if (callback)
    {
        callback(frame, result);
    }

    return result;
}

bool RealtimePipeline::waitForAttitude(
    double host_time,
    kalfilter::GimbalSample& sample)
{
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double, std::milli>(
                std::max(0.0, config_.timing.attitude_wait_ms)));

    for (;;)
    {
        if (history_.interpolate(host_time, sample)) return true;
        if (!running_.load()) return false;
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(
            std::chrono::milliseconds(1));
    }
}

void RealtimePipeline::feedbackLoop(std::stop_token stop_token)
{
    comm::GimbalToVision packet;

    while (running_.load() && !stop_token.stop_requested())
    {
        bool received = false;
        while (serial_.poll(packet))
        {
            received = true;
            handleFeedback(packet);
        }

        if (!received)
        {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(1));
        }
    }
}

void RealtimePipeline::handleFeedback(
    const comm::GimbalToVision& packet)
{
    const double receive_time = steadyNowSeconds();

    uint64_t board_time_us {};
    double event_host_time {};
    if (!clock_.observeAndMap(
            packet.DWT_stamp,
            receive_time,
            board_time_us,
            event_host_time))
    {
        return;
    }

    kalfilter::GimbalSample sample;
    sample.host_time = event_host_time;
    sample.board_time_us = board_time_us;
    sample.q = Eigen::Quaterniond(
        packet.q[0],
        packet.q[1],
        packet.q[2],
        packet.q[3]);
    sample.pitch = packet.pitch;
    sample.pitch_vel = packet.pitch_vel;
    sample.yaw = packet.yaw;
    sample.yaw_vel = packet.yaw_vel;
    sample.bullet_speed = packet.bullet_speed;
    sample.mode = packet.mode;

    if (!history_.push(sample)) return;

    last_feedback_receive_time_.store(receive_time);
    std::lock_guard<std::mutex> lock(stats_mutex_);
    ++stats_.received_feedback;
}

bool RealtimePipeline::fail(
    const std::string& message,
    std::string* error)
{
    last_error_ = message;
    if (error) *error = message;
    return false;
}

}   // namespace realtime
