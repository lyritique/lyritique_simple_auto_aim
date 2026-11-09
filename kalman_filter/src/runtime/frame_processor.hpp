# ifndef AUTOAIM_RUNTIME_FRAME_PROCESSOR_HPP
# define AUTOAIM_RUNTIME_FRAME_PROCESSOR_HPP

# include <array>
# include <string>
# include <vector>

# include <Eigen/Geometry>
# include <opencv2/core.hpp>

# include "core/config.hpp"
# include "core/frame.hpp"
# include "estimation/kalman_filter.hpp"
# include "vision/armor_detector.hpp"
# include "vision/pnp_solver.hpp"

namespace realtime
{

struct FrameObservation
{
    detector::ArmorDetection detection;
    std::array<cv::Point2f, 4> refined_corners {};
    pnp::PnPResult pose;
    bool sent_to_ekf {};
};

struct FrameProcessingResult
{
    detector::ArmorDetections detections;
    std::vector<FrameObservation> observations;
    std::vector<double> reprojection_errors;
    kalfilter::KFResult state;
    bool ekf_initialized {};
    bool state_valid {};
};

class FrameProcessor
{
public:
    explicit FrameProcessor(const core::AppConfig& config);

    void reset();

    // R_C0C 把当前相机系观测转到算法世界 C0。
    FrameProcessingResult process(
        const core::TimestampedFrame& frame,
        const Eigen::Matrix3d& R_C0C);

    const cv::Mat& cameraMatrix() const noexcept
    {
        return pnp_.cameraMatrix();
    }

    const cv::Mat& distortionCoeffs() const noexcept
    {
        return pnp_.distortionCoeffs();
    }

private:
    core::DetectorConfig detector_config_;
    double target_max_distance_ {};

    detector::Detector detector_;
    pnp::pnpSolver pnp_;
    kalfilter::KalmanFilter ekf_;

    bool ekf_initialized_ {};
    bool has_last_frame_time_ {};
    double last_frame_time_ {};
};

}   // namespace realtime

# endif
