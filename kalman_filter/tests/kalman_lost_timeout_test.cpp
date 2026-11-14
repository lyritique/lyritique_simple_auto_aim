# include "estimation/kalman_filter.hpp"

# include <cstdio>

namespace
{

int failures = 0;

void expect(bool condition, const char* what)
{
    if (!condition)
    {
        std::printf("失败: %s\n", what);
        ++failures;
    }
}

}   // namespace

// 作用：确认丢目标后状态会失效，别一直拿预测值当真实状态。实现：初始化后只 predict 不 update，看 valid 何时变假。
int main()
{
    kalfilter::KalmanConfig config;
    config.lost_timeout = 0.3;

    kalfilter::KalmanFilter filter(config);

    kalfilter::armerState measurement;
    measurement.measurement << 0.0, 0.0, 2.0, 0.0;
    filter.init(measurement);

    // 刚初始化，状态有效。
    expect(filter.getOutput().valid, "初始化后应为 valid");

    // 4 帧 × 50 ms = 0.2 s，还在容忍范围内。
    for (int i = 0; i < 4; ++i) filter.predict(0.05);
    expect(filter.getOutput().valid, "丢目标 0.2s 内仍应 valid");

    // 再 4 帧到 0.4 s，超过 0.3 s 阈值，必须失效。
    for (int i = 0; i < 4; ++i) filter.predict(0.05);
    expect(!filter.getOutput().valid, "丢目标超过阈值后应为 invalid");

    // 重新观测到目标后要能恢复。
    measurement.measurement << 0.0, 0.0, 2.0, 0.0;
    filter.update(
        measurement,
        kalfilter::cam_ypd {0.0, 0.0, 2.0},
        Eigen::Matrix3d::Identity());
    expect(filter.getOutput().valid, "重新观测后应恢复 valid");

    if (failures == 0)
    {
        std::printf("kalman_lost_timeout_test 通过\n");
        return 0;
    }
    return 1;
}
