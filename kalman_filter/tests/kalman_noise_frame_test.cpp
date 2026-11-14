# include "estimation/kalman_filter.hpp"

# include <cmath>
# include <cstdio>

namespace
{

int failures = 0;

void expect(bool condition, const char* what, double value)
{
    if (!condition)
    {
        std::printf("失败: %s (%.9f)\n", what, value);
        ++failures;
    }
}

kalfilter::armerState makeMeasurement(const Eigen::Vector3d& p, double yaw)
{
    kalfilter::armerState state;
    state.measurement << p.x(), p.y(), p.z(), yaw;
    state.testResult.id = 0;
    return state;
}

// 相机系观测折成 YPD，和 frame_processor 里的 computeYPD 一致。
kalfilter::cam_ypd makeYpd(const Eigen::Vector3d& p)
{
    kalfilter::cam_ypd ypd;
    ypd.azi = std::atan2(p.x(), p.z());
    ypd.ele = std::atan2(p.y(), std::hypot(p.x(), p.z()));
    ypd.dist = p.norm();
    return ypd;
}

}   // namespace

/*
 * 作用：确认观测噪声 R 会跟着 R_C0C 一起旋转。
 * 实现：同一份相机系观测喂给两个滤波器，一个 R_C0C = I，一个 R_C0C = Rz(θ)。
 *       正确的话，后者的状态应该正好是前者绕 z 轴转 θ 的结果；R 没转就会出现偏差。
 */
int main()
{
    const double theta = 1.2;   // 约 69°，明显偏离初始姿态
    const Eigen::Matrix3d R_C0C =
        Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY()).toRotationMatrix();

    kalfilter::KalmanConfig config;
    kalfilter::KalmanFilter base(config);
    kalfilter::KalmanFilter rotated(config);

    // 相机系下的目标位置和朝向（两个滤波器看到的是同一份相机系观测）。
    Eigen::Vector3d p_cam(0.30, -0.10, 2.00);
    const double yaw_cam = 0.40;

    auto init_base = makeMeasurement(p_cam, yaw_cam);
    auto init_rotated = makeMeasurement(R_C0C * p_cam, yaw_cam + theta);
    base.init(init_base);
    rotated.init(init_rotated);

    // 让目标在相机系里小幅移动，跑若干帧后比较。
    for (int i = 0; i < 30; ++i)
    {
        p_cam += Eigen::Vector3d(0.004, 0.002, -0.01);
        const double yaw = yaw_cam + 0.01 * i;
        const auto ypd = makeYpd(p_cam);

        auto obs_base = makeMeasurement(p_cam, yaw);
        auto obs_rotated =
            makeMeasurement(R_C0C * p_cam, yaw + theta);

        base.update(obs_base, ypd, Eigen::Matrix3d::Identity());
        rotated.update(obs_rotated, ypd, R_C0C);
    }

    const auto a = base.getOutput();
    const auto b = rotated.getOutput();
    expect(a.valid && b.valid, "两个滤波器都应有效", 0.0);

    const Eigen::Vector3d pa(a.state(0), a.state(1), a.state(2));
    const Eigen::Vector3d pb(b.state(0), b.state(1), b.state(2));
    const double pos_err = (R_C0C * pa - pb).norm();
    expect(pos_err < 1e-6, "旋转后的位置应保持一致", pos_err);

    // 位置是关键判据：R 没旋转时这里的误差是 0.33 m 量级。
    // 朝向只有 1e-5 rad 的残差，来自协方差交叉项带来的数值差异，与坐标系无关。
    double yaw_err = a.state(3) + theta - b.state(3);
    while (yaw_err > M_PI) yaw_err -= 2.0 * M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;
    expect(std::abs(yaw_err) < 1e-6, "旋转后的朝向应保持一致", yaw_err);

    if (failures == 0)
    {
        std::printf("kalman_noise_frame_test 通过\n");
        return 0;
    }
    return 1;
}
