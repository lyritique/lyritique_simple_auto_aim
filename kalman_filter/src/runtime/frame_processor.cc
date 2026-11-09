# include "runtime/frame_processor.hpp"

# include <opencv2/imgproc.hpp>

# include <algorithm>
# include <cmath>
# include <iostream>
# include <utility>

namespace realtime
{
namespace
{

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

FrameProcessor::FrameProcessor(const core::AppConfig& config)
    : detector_config_(config.detector),
      target_max_distance_(config.target_max_distance),
      detector_(detector_config_)
{
}

void FrameProcessor::reset()
{
    ekf_.reset();
    ekf_initialized_ = false;
    has_last_frame_time_ = false;
    last_frame_time_ = 0.0;
}

FrameProcessingResult FrameProcessor::process(
    const core::TimestampedFrame& frame,
    const Eigen::Matrix3d& R_C0C)
{
    FrameProcessingResult result;

    if (ekf_initialized_ && has_last_frame_time_)
    {
        const double dt = frame.timestamp - last_frame_time_;
        if (std::isfinite(dt) && dt > 0.0)
        {
            ekf_.predict(dt);
        }
    }

    last_frame_time_ = frame.timestamp;
    has_last_frame_time_ = true;

    cv::Mat gray;
    cv::cvtColor(frame.bgr, gray, cv::COLOR_BGR2GRAY);
    result.detections = detector_.detect(frame.bgr);

    for (const auto& detection : result.detections)
    {
        const double edge_length = cv::norm(
            detection.corners[0] - detection.corners[1]);
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

        FrameObservation observation;
        observation.detection = detection;
        observation.refined_corners = refined;
        observation.pose = pose;
        result.reprojection_errors.push_back(pose.err);

        kalfilter::cam_ypd ypd;
        computeYPD(pose.measurement, ypd);
        if (ypd.dist > target_max_distance_)
        {
            result.observations.push_back(observation);
            continue;
        }

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

        observation.sent_to_ekf = true;
        result.observations.push_back(observation);

        if (!ekf_initialized_)
        {
            ekf_.init(ekf_input);
            ekf_initialized_ = true;
            break;
        }

        ekf_.update(ekf_input, ypd);
        std::cout << "\n";
    }

    result.ekf_initialized = ekf_initialized_;
    if (ekf_initialized_)
    {
        ekf_.renewArmorList();
        result.state = ekf_.getOutput();
        result.state_valid = result.state.valid;
    }

    return result;
}

}   // namespace realtime
