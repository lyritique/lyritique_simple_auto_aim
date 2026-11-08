# include "realtime_pipeline.hpp"

# include <opencv2/imgproc.hpp>

# include <algorithm>
# include <chrono>
# include <cmath>
# include <limits>
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

void computeYPD(
    const Eigen::Matrix<double, 4, 1>& measurement,
    kalfilter::cam_ypd& ypd)
{
    ypd.azi = std::atan2(measurement[0], measurement[2]);
    ypd.ele = std::atan2(
        measurement[1],
        std::hypot(measurement[0], measurement[2]));
    ypd.dist = std::sqrt(
        measurement[0] * measurement[0] +
        measurement[1] * measurement[1] +
        measurement[2] * measurement[2]);
}

std::array<cv::Point2f, 4> refineArmorCorners(
    const cv::Mat& gray,
    const std::array<cv::Point2f, 4>& input,
    float max_shift,
    cv::Size window_size)
{
    std::vector<cv::Point2f> refined(input.begin(), input.end());
    const std::vector<cv::Point2f> original = refined;

    cv::cornerSubPix(
        gray,
        refined,
        window_size,
        cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
            20,
            0.01));

    std::array<cv::Point2f, 4> result {};
    for (std::size_t i = 0; i < refined.size(); ++i)
    {
        const cv::Point2f delta = refined[i] - original[i];
        const bool inside =
            refined[i].x >= 1.5F &&
            refined[i].y >= 1.5F &&
            refined[i].x < gray.cols - 1.5F &&
            refined[i].y < gray.rows - 1.5F;
        const bool reasonable =
            std::isfinite(refined[i].x) &&
            std::isfinite(refined[i].y) &&
            cv::norm(delta) <= max_shift;
        result[i] = inside && reasonable ? refined[i] : original[i];
    }
    return result;
}

}   // namespace

RealtimePipeline::RealtimePipeline(RealtimePipelineConfig config)
    : config_(std::move(config)),
      clock_(config_.comm_latency),
      history_(
          config_.max_history_seconds,
          config_.max_attitude_gap_seconds,
          config_.max_attitude_prediction_seconds),
      detector_(config_.detector)
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
    clock_.setCommLatency(config_.comm_latency);
    history_.clear();
    latency_.clear();
    latency_.recordCommunication(config_.comm_latency);

    ekf_.reset();
    ekf_initialized_ = false;
    has_last_frame_time_ = false;
    last_frame_time_ = 0.0;
    has_q_init_ = false;
    q_init_.setIdentity();
    last_feedback_receive_time_.store(0.0);

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_ = PipelineStats {};
    }

    if (config_.serial_device.empty())
    {
        return fail("串口设备路径为空", error);
    }

    if (!serial_.open(
            config_.serial_device.c_str(),
            config_.serial_baud))
    {
        return fail(
            "串口打开失败: " + config_.serial_device +
                " @" + std::to_string(config_.serial_baud),
            error);
    }

    if (!camera_.open(config_.camera, error))
    {
        serial_.close();
        camera_.close();
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
        camera::CameraFrame frame;
        std::string error;
        if (!camera_.read(frame, &error))
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

    camera_.close();
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
    const camera::CameraFrame& frame,
    const FrameCallback& callback)
{
    PipelineFrameResult result;
    result.frame_time = frame.host_time;

    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        ++stats_.processed_frames;
    }

    kalfilter::GimbalSample attitude;
    if (!waitForAttitude(frame.host_time, attitude))
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

    if (ekf_initialized_ && has_last_frame_time_)
    {
        const double dt = frame.host_time - last_frame_time_;
        if (std::isfinite(dt) && dt > 0.0)
        {
            ekf_.predict(dt);
        }
    }

    last_frame_time_ = frame.host_time;
    has_last_frame_time_ = true;

    cv::Mat gray;
    cv::cvtColor(frame.bgr, gray, cv::COLOR_BGR2GRAY);
    auto detections = detector_.detect(frame.bgr);

    for (const auto& detection : detections)
    {
        const double edge_length =
            cv::norm(detection.corners[0] - detection.corners[1]);
        const int half_window = std::clamp(
            static_cast<int>(
                std::lround(edge_length * 0.08F)),
            2,
            5);

        const auto refined = refineArmorCorners(
            gray,
            detection.corners,
            2.0F,
            cv::Size(half_window, half_window));

        pnp_.input(detection.corners, detection.big);
        pnp_.easySolver();
        const pnp::PnPResult raw_pose = pnp_.getOutput();

        pnp_.input(refined, detection.big);
        pnp_.easySolver();
        const pnp::PnPResult refined_pose = pnp_.getOutput();

        const bool use_refined =
            refined_pose.valid &&
            (!raw_pose.valid ||
             refined_pose.err + 0.03 < raw_pose.err);
        const pnp::PnPResult pose =
            use_refined ? refined_pose : raw_pose;
        if (!pose.valid) continue;

        kalfilter::cam_ypd ypd;
        computeYPD(pose.measurement, ypd);
        if (ypd.dist > config_.target_max_distance) continue;

        const Eigen::Vector3d p_cam(
            pose.measurement(0),
            pose.measurement(1),
            pose.measurement(2));
        const Eigen::Vector3d p_w = R_C0C * p_cam;
        const Eigen::Vector3d n_w = R_C0C * pose.normal;

        kalfilter::armerState ekf_input {};
        ekf_input.measurement <<
            p_w.x(),
            p_w.y(),
            p_w.z(),
            std::atan2(n_w.x(), n_w.z());

        if (!ekf_initialized_)
        {
            ekf_.init(ekf_input);
            ekf_initialized_ = true;
        }
        else
        {
            ekf_.update(ekf_input, ypd);
        }

        ++result.valid_detections;
    }

    if (ekf_initialized_)
    {
        ekf_.renewArmorList();
        const kalfilter::KFResult state = ekf_.getOutput();
        result.state = state;
        result.state_valid = state.valid;

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            latest_state_ = state;
        }

        if (state.valid)
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            ++stats_.valid_frames;
        }
    }

    const double command_ready_time = steadyNowSeconds();
    result.process_latency =
        command_ready_time - frame.host_time;

    kalfilter::FrameTiming timing;
    timing.exposure_time = frame.host_time;
    timing.frame_received_time = frame.host_time;
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
                std::max(0.0, config_.attitude_wait_ms)));

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
