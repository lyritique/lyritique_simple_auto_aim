# include "runtime/frame_processor.hpp"

# include <opencv2/imgproc.hpp>

# include <algorithm>
# include <chrono>
# include <cmath>
# include <iostream>
# include <utility>

namespace realtime
{
namespace
{

// 作用：把相机系 XYZ 测量折成方位角、俯仰角、距离，喂给 EKF 用。实现：atan2 求水平角，hypot 求俯仰角，欧氏范数求距离。
void computeYPD(
    const Eigen::Matrix<double, 4, 1>& measurement,
    kalfilter::cam_ypd& ypd)
{
    // 作用：水平方位角，x 与 z 的夹角。实现：atan2(x, z)，自带象限判断。
    ypd.azi = std::atan2(measurement[0], measurement[2]);
    // 作用：俯仰角，用 xz 平面距离当前分母更稳。实现：atan2(y, hypot(x, z))。
    ypd.ele = std::atan2(
        measurement[1],
        std::hypot(measurement[0], measurement[2]));
    // 作用：直线距离，后面的距离门限要用。实现：三维平方和开根。
    ypd.dist = std::sqrt(
        measurement[0] * measurement[0] +
        measurement[1] * measurement[1] +
        measurement[2] * measurement[2]);
}

// 作用：把检测框的四个角点往亚像素精度再抠一抠。实现：cornerSubPix 迭代细化，再用“在画面内且位移合理”两道闸决定采不采信。
std::array<cv::Point2f, 4> refineArmorCorners(
    const cv::Mat& gray,
    const std::array<cv::Point2f, 4>& input,
    float max_shift,
    cv::Size window_size)
{
    // 作用：cornerSubPix 只认 vector，先换个容器。实现：用输入的首尾迭代器构造。
    std::vector<cv::Point2f> refined(input.begin(), input.end());
    // 作用：留一份原始坐标当保底，细化了不满意还能退回去。实现：拷一份到 original。
    const std::vector<cv::Point2f> original = refined;

    // 作用：在灰度图上做亚像素角点搜索。实现：窗口用传入尺寸，死区 -1 表示不设，迭代上限 20 次或步长小于 0.01 就停。
    cv::cornerSubPix(
        gray,
        refined,
        window_size,
        cv::Size(-1, -1),
        cv::TermCriteria(
            cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
            20,
            0.01));

    // 作用：装最终结果，默认全零。实现：固定大小数组，逐点填。
    std::array<cv::Point2f, 4> result {};
    // 作用：逐点验收细化成果，不合格就退回原值。实现：四个点循环，分别判断是否越界、是否飞了。
    for (std::size_t i = 0; i < refined.size(); ++i)
    {
        // 作用：算细化前后挪了多少，用来判断有没有“跑飞”。实现：两个点相减。
        const cv::Point2f delta = refined[i] - original[i];
        // 作用：确认点还在画面里，别贴着边缘让 cornerSubPix 编数。实现：四边各留 1.5 像素余量。
        const bool inside =
            refined[i].x >= 1.5F &&
            refined[i].y >= 1.5F &&
            refined[i].x < gray.cols - 1.5F &&
            refined[i].y < gray.rows - 1.5F;
        // 作用：确认坐标有限且位移没超预算，超了就说明细化不靠谱。实现：isfinite 加位移范数比较。
        const bool reasonable =
            std::isfinite(refined[i].x) &&
            std::isfinite(refined[i].y) &&
            cv::norm(delta) <= max_shift;
        // 作用：两道闸都过才用新坐标，否则老老实实用原值。实现：三目运算符一次定生死。
        result[i] = inside && reasonable ? refined[i] : original[i];
    }
    return result;
}

}   // namespace

// 作用：按配置把处理器组装起来，顺便存下距离阈值。实现：初始化列表构造检测器，其余成员就地默认构造。
FrameProcessor::FrameProcessor(const core::AppConfig& config)
    : detector_config_(config.detector),
      intrinsics_(config.intrinsics),
      target_max_distance_(config.target_max_distance),
      profile_(config.profile),
      detector_(detector_config_),
      pnp_(intrinsics_)
{
}

// 作用：重置滤波器和帧时间记忆，开始新一段视频。实现：ekf_.reset() 加标志位归零。
void FrameProcessor::reset()
{
    ekf_.reset();
    ekf_initialized_ = false;
    size_warned_ = false;
    has_last_frame_time_ = false;
    last_frame_time_ = 0.0;
}

// 作用：只跑检测，不碰云台姿态。实现：尺寸检查一次，然后交给检测器。
// 单独拆出来是为了让实时链路先做检测，把等 IMU 的时间盖在推理下面。
detector::ArmorDetections FrameProcessor::detect(
    const core::TimestampedFrame& frame)
{
    // 作用：画面尺寸和内参必须配套，不匹配就提醒一次，后面 PnP 的尺度都会歪。实现：比较行列数，只警告不拦帧。
    if (!size_warned_ &&
        (frame.bgr.cols != intrinsics_.width ||
         frame.bgr.rows != intrinsics_.height))
    {
        size_warned_ = true;
        std::cerr << "帧尺寸 " << frame.bgr.cols << "x" << frame.bgr.rows
                  << " 与内参 " << intrinsics_.width << "x"
                  << intrinsics_.height
                  << " 不一致，PnP 结果可能不准\n";
    }

    return detector_.detect(frame.bgr);
}

// 作用：拿检测结果做客位姿和滤波。实现：补一次预测，逐块细化 + PnP，按距离筛选后送进 EKF。
FrameProcessingResult FrameProcessor::solve(
    const core::TimestampedFrame& frame,
    const detector::ArmorDetections& detections,
    const Eigen::Matrix3d& R_C0C)
{
    // 作用：先把这一帧的结果容器准备好，各字段默认零值。实现：栈上默认构造。
    FrameProcessingResult result;
    result.detections = detections;
    const auto profile_frame_start = std::chrono::steady_clock::now();

    // 作用：只有上过战场才知道该走多远，首帧不预测。实现：已初始化且有上帧时间才继续。
    if (ekf_initialized_ && has_last_frame_time_)
    {
        // 作用：算两帧间隔，作为运动模型的步长。实现：当前时间戳减上一帧。
        const double dt = frame.timestamp - last_frame_time_;
        // 作用：dt 必须正常，否则预测等于扔骰子。实现：有限且为正才调用 ekf_.predict。
        if (std::isfinite(dt) && dt > 0.0)
        {
            ekf_.predict(dt);
        }
    }

    // 作用：更新帧时间记忆，给下一帧算 dt 用。实现：覆盖时间、置有效标志。
    last_frame_time_ = frame.timestamp;
    has_last_frame_time_ = true;

    // 作用：准备灰度图，亚像素细化只吃它。实现：cvtColor 做 BGR2GRAY。
    cv::Mat gray;
    cv::cvtColor(frame.bgr, gray, cv::COLOR_BGR2GRAY);

    // 作用：逐块板子过堂，能算位姿的才留。实现：遍历检测结果，内部完成细化、PnP、距离筛选和 EKF 投喂。
    for (const auto& detection : result.detections)
    {
        // 作用：用第一条边估板子多大，决定细化窗口开多大。实现：两个角点求范数。
        const double edge_length = cv::norm(
            detection.corners[0] - detection.corners[1]);
        // 作用：按板子尺寸自适应窗口，近处开大点、远处开小点。实现：边长乘 0.08 取整后夹到 [2,5]。
        const int half_window = std::clamp(
            static_cast<int>(
                std::lround(edge_length * 0.08F)),
            2,
            5);

        // 作用：拿到亚像素角点，最多允许挪 2 像素。实现：调 refineArmorCorners，窗口用上面的自适应尺寸。
        const auto refined = refineArmorCorners(
            gray,
            detection.corners,
            2.0F,
            cv::Size(half_window, half_window));

        // 作用：先用原始角点算一版位姿，当对照组。实现：input + easySolver + getOutput 三连。
        pnp_.input(detection.corners, detection.big);
        pnp_.easySolver();
        const pnp::PnPResult raw_pose = pnp_.getOutput();

        // 作用：再用细化角点算一版，看看能不能更好。实现：同样三连，只是换个角点输入。
        pnp_.input(refined, detection.big);
        pnp_.easySolver();
        const pnp::PnPResult refined_pose = pnp_.getOutput();

        // 作用：择优录取：细化版有效且误差明显更小才用它。实现：误差要领先 0.03 才换，防止两版半斤八两瞎折腾。
        const bool use_refined =
            refined_pose.valid &&
            (!raw_pose.valid ||
             refined_pose.err + 0.03 < raw_pose.err);
        // 作用：定下最终位姿，后面都基于它。实现：三目选一个。
        const pnp::PnPResult pose =
            use_refined ? refined_pose : raw_pose;
        // 作用：两版都失败就放弃这块板子，别拿垃圾喂滤波。实现：invalid 直接 continue。
        if (!pose.valid) continue;

        // 作用：把这块板子的处理账本先立起来，检测、角点、位姿都记上。实现：逐字段赋值。
        FrameObservation observation;
        observation.detection = detection;
        observation.refined_corners = refined;
        observation.pose = pose;
        // 作用：顺手记一条重投影误差，回放时能看到解算质量。实现：push_back pose.err。
        result.reprojection_errors.push_back(pose.err);

        // 作用：把位姿折成 YPD 观测，同时拿距离做门限判断。实现：先声明局部变量再调 computeYPD。
        kalfilter::cam_ypd ypd;
        computeYPD(pose.measurement, ypd);
        observation.ypd = ypd;
        // 作用：太远的板子只记账不滤波，免得噪声把状态带歪。实现：距离超阈值就 push 后 continue。
        if (ypd.dist > target_max_distance_)
        {
            result.observations.push_back(observation);
            continue;
        }

        // 作用：把相机系位置拼成向量，准备旋转到世界系。实现：从 measurement 前三项构造 Vector3d。
        const Eigen::Vector3d p_cam(
            pose.measurement(0),
            pose.measurement(1),
            pose.measurement(2));
        // 作用：位置从相机系转到算法世界 C0。实现：乘旋转矩阵 R_C0C。
        const Eigen::Vector3d p_w = R_C0C * p_cam;
        // 作用：法向量也一起转，否则 yaw 观测会错位。实现：同一次旋转作用在 normal 上。
        const Eigen::Vector3d n_w = R_C0C * pose.normal;

        // 作用：凑一份 EKF 观测：世界系位置加装甲板朝向角。实现：聚合初始化后逐项填 measurement。
        kalfilter::armerState ekf_input {};
        ekf_input.measurement <<
            p_w.x(),
            p_w.y(),
            p_w.z(),
            std::atan2(n_w.x(), n_w.z());
        observation.ekf_yaw = ekf_input.measurement(3);

        // 作用：先把“已送 EKF”的章盖好，再入账。实现：标记置 true，然后 push 观测。
        observation.sent_to_ekf = true;
        result.observations.push_back(observation);

        // 作用：滤波器还没出生就先给它第一口饭。实现：初次观测调 init，置标志后直接 break，本帧不再 update。
        if (!ekf_initialized_)
        {
            ekf_.init(ekf_input);
            ekf_initialized_ = true;
            break;
        }

        // 作用：已初始化就正常更新，只拿一块板子更新一次，避免同帧重复喂。实现：update 后打个空行当调试分隔。
        ekf_.update(ekf_input, ypd, R_C0C);
    }

    // 作用：把初始化状态汇报出去，外部据此判断要不要信任结果。实现：拷贝成员标志。
    result.ekf_initialized = ekf_initialized_;
    // 作用：滤波器活着就顺带刷新装甲板列表并导出状态。实现：renewArmorList 后取 getOutput，valid 一并透传。
    if (ekf_initialized_)
    {
        ekf_.renewArmorList();
        result.state = ekf_.getOutput();
        result.state_valid = result.state.valid;
    }

    // 作用：每 50 帧打一次分段平均，判断该砍推理还是砍后处理。实现：累加后打印并清零。
    if (profile_)
    {
        const auto to_ms = [](const auto& from, const auto& to)
        {
            return std::chrono::duration<double, std::milli>(to - from).count();
        };
        // 检测耗时来自检测器自己记的三段，解算耗时取 solve 的墙钟。
        const double detect_ms = detector_.timings().preprocess_ms +
            detector_.timings().inference_ms +
            detector_.timings().postprocess_ms;
        const double solve_ms =
            to_ms(profile_frame_start, std::chrono::steady_clock::now());

        profile_detect_ += detect_ms;
        profile_pre_ += detector_.timings().preprocess_ms;
        profile_inf_ += detector_.timings().inference_ms;
        profile_post_ += detector_.timings().postprocess_ms;
        profile_rest_ += solve_ms;
        ++profile_frames_;

        if (profile_frames_ >= 50)
        {
            const double n = static_cast<double>(profile_frames_);
            std::clog << "[profile] detect=" << profile_detect_ / n
                      << "ms (pre " << profile_pre_ / n
                      << " / inf " << profile_inf_ / n
                      << " / post " << profile_post_ / n
                      << ") solve=" << profile_rest_ / n
                      << "ms total=" << (profile_detect_ + profile_rest_) / n
                      << "ms\n";
            profile_frames_ = 0;
            profile_detect_ = profile_pre_ = profile_inf_ = 0.0;
            profile_post_ = profile_rest_ = 0.0;
        }
    }

    return result;
}

// 作用：回放等场景一把梭，检测和解算连着做。实现：先 detect 再 solve。
FrameProcessingResult FrameProcessor::process(
    const core::TimestampedFrame& frame,
    const Eigen::Matrix3d& R_C0C)
{
    return solve(frame, detect(frame), R_C0C);
}

}   // namespace realtime
