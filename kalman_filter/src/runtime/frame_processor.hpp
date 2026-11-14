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


//YPD 换算、亚像素角点细化与双 PnP 择优、世界系旋转、EKF init/update 分支


namespace realtime
{

struct FrameObservation
{
    // 作用：检测器交出来的这块装甲板，原始信息全留着。实现：整条 ArmorDetection 拷贝一份，方便回溯。
    detector::ArmorDetection detection;
    // 作用：亚像素细化后的四角，PnP 更待见它。实现：固定 4 点数组，和检测框一一对应。
    std::array<cv::Point2f, 4> refined_corners {};
    // 作用：这块板子最终采用的位姿和解算误差。实现：PnPResult 整包存，valid 字段说明能不能用。
    pnp::PnPResult pose;
    // 作用：记录这块板子有没有真的进 EKF。实现：bool 标记，供下游统计和调试。
    bool sent_to_ekf {};
    // 作用：折成的 YPD 观测，标定观测噪声 R 用。实现：computeYPD 的结果原样带上。
    kalfilter::cam_ypd ypd {};
    // 作用：实际喂给 EKF 的装甲板朝向角。实现：由板法向 atan2 得到。
    double ekf_yaw {};
};

struct FrameProcessingResult
{
    // 作用：这一帧所有检测框，画图用。实现：检测器的输出原样带回。
    detector::ArmorDetections detections;
    // 作用：每块板子的细粒度处理账本。实现：逐块收集，没通过 PnP 的不进这个列表。
    std::vector<FrameObservation> observations;
    // 作用：各块板子的重投影误差，衡量解算质量。实现：每块 push 一个 pose.err。
    std::vector<double> reprojection_errors;
    // 作用：EKF 这一帧吐出的最终状态，外面就等它。实现：KFResult 整包，valid 说明可信与否。
    kalfilter::KFResult state;
    // 作用：告诉调用方 EKF 现在有没有起来。实现：透传成员 ekf_initialized_。
    bool ekf_initialized {};
    // 作用：告诉调用方 state 能不能信。实现：透传 state.valid。
    bool state_valid {};
};

class FrameProcessor
{
public:
    // 作用：按配置把检测器、PnP、EKF 都装配好。实现：初始化列表里拷贝配置并构造检测器，EKF 用默认构造。
    // 参数：config —— 全局配置，只读借用，内部只挑 detector 和 target_max_distance 用。
    // 返回：无。此时 EKF 还未初始化，要等第一帧观测来 init。
    explicit FrameProcessor(const core::AppConfig& config);

    // 作用：把 EKF 和帧时间记忆全清掉，换视频或重连时用。实现：调 ekf_.reset() 再归零几个标志位。
    // 参数：无。返回：无。清完就和刚构造出来一样干净。
    void reset();

    // R_C0C 把当前相机系观测转到算法世界 C0。
    // 作用：一帧图像进来，走完检测、PnP、坐标旋转、EKF 更新。实现：先预测、再检测、逐块解算择优，最后更新并取状态。
    // 参数：frame —— 带时间戳的 BGR 帧；R_C0C —— 相机系到算法世界的旋转，正交矩阵，调用方保证。
    // 返回：本帧的检测、观测账本和滤波状态；EKF 没初始化时 state 字段无效，看 ekf_initialized。
    FrameProcessingResult process(
        const core::TimestampedFrame& frame,
        const Eigen::Matrix3d& R_C0C);

    // 作用：把相机内参借给外面画重投影。实现：直接转发 pnp_ 里的 cv::Mat，不改不拷贝。
    // 参数：无。返回：3x3 内参矩阵的常量引用，生命周期跟本对象走，别存指针。
    const cv::Mat& cameraMatrix() const noexcept
    {
        return pnp_.cameraMatrix();
    }

    // 作用：把畸变系数借给外面，画图时去畸变用。实现：同样直接转发。
    // 参数：无。返回：畸变系数向量的常量引用，同样别越过对象生命周期。
    const cv::Mat& distortionCoeffs() const noexcept
    {
        return pnp_.distortionCoeffs();
    }

private:
    // 作用：检测器参数的本地副本，检测器生命周期都靠它。实现：构造时从 AppConfig 拷一份。
    core::DetectorConfig detector_config_;
    // 作用：内参的本地副本，用来校验每一帧的尺寸和它是否匹配。实现：构造时从 AppConfig 拷一份。
    core::IntrinsicsConfig intrinsics_;
    // 作用：超过这个距离的板子不进 EKF，防止拿远处的噪声当目标。实现：double 阈值，process 里和 ypd.dist 比。
    double target_max_distance_ {};

    // 作用：性能统计开关和累计器。实现：每帧累加，够 50 帧打一次平均再清零。
    bool profile_ {};
    int profile_frames_ {};
    double profile_detect_ {};
    double profile_pre_ {};
    double profile_inf_ {};
    double profile_post_ {};
    double profile_rest_ {};

    // 作用：真正干检测的工人。实现：构造时用 detector_config_ 初始化。
    detector::Detector detector_;
    // 作用：PnP 解算器，负责把像素角点变成 3D 位姿。实现：构造时用 intrinsics_ 初始化，不再吃类内硬编码。
    pnp::pnpSolver pnp_;
    // 作用：卡尔曼滤波器，整条链路的大脑。实现：默认构造，第一次观测时 init。
    kalfilter::KalmanFilter ekf_;

    // 作用：记住 EKF 是否已经初始化，没初始化就只能 init 不能 update。实现：bool 开关，init 成功后置 true。
    bool ekf_initialized_ {};
    // 作用：画面尺寸和内参对不上的告警只打一次，避免每帧刷屏。实现：bool 开关。
    bool size_warned_ {};
    // 作用：记住有没有上一帧时间，第一帧没有 dt 可算。实现：bool 门禁，首帧后置 true。
    bool has_last_frame_time_ {};
    // 作用：上一帧的时间戳，用来算预测步的 dt。实现：double 秒，每帧覆盖。
    double last_frame_time_ {};
};

}   // namespace realtime

# endif
