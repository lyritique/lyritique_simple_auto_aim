#include "timing/gimbal_history.hpp"

#include <cmath>
#include <iostream>

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

kalfilter::GimbalSample makeSample(double time, double angle) {
  kalfilter::GimbalSample sample;
  sample.host_time = time;
  sample.board_time_us = static_cast<uint64_t>(time * 1'000'000.0);
  sample.q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()));
  sample.pitch = static_cast<float>(angle);
  sample.pitch_vel = 0.0F;
  sample.yaw = static_cast<float>(angle);
  sample.yaw_vel = static_cast<float>(10.0 * kDeg / 0.01);
  sample.bullet_speed = 25.0F;
  sample.mode = 2;
  return sample;
}

double quaternionAngle(const Eigen::Quaterniond& q) { return std::abs(Eigen::AngleAxisd(q).angle()); }

}  // namespace

int main() {
  kalfilter::GimbalFeedbackHistory history(2.0, 0.1, 0.05);
  if (!history.push(makeSample(100.000, 0.0)) || !history.push(makeSample(100.010, 10.0 * kDeg))) {
    std::cerr << "history push failed\n";
    return 1;
  }

  kalfilter::GimbalSample output;
  if (!history.interpolate(100.005, output) || std::abs(output.yaw - 5.0F * kDeg) > 1e-4F ||
      std::abs(quaternionAngle(output.q) - 5.0 * kDeg) > 1e-6) {
    std::cerr << "history interpolation failed\n";
    return 2;
  }

  if (history.interpolate(99.999, output) || history.interpolate(100.011, output)) {
    std::cerr << "out-of-range interpolation accepted\n";
    return 3;
  }

  if (!history.predict(100.015, output) || std::abs(output.yaw - 15.0F * kDeg) > 1e-4F ||
      std::abs(quaternionAngle(output.q) - 15.0 * kDeg) > 1e-6) {
    std::cerr << "history prediction failed\n";
    return 4;
  }

  if (history.predict(100.061, output)) {
    std::cerr << "prediction horizon not limited\n";
    return 5;
  }

  if (history.push(makeSample(100.005, 0.0))) {
    std::cerr << "out-of-order sample accepted\n";
    return 6;
  }

  kalfilter::GimbalFeedbackHistory gapped(2.0, 0.02, 0.05);
  gapped.push(makeSample(1.0, 0.0));
  gapped.push(makeSample(1.1, 1.0));
  if (gapped.interpolate(1.05, output)) {
    std::cerr << "large interpolation gap accepted\n";
    return 7;
  }

  std::cout << "gimbal history test passed\n";
  return 0;
}
