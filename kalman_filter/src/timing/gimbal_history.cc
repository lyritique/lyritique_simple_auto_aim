# include "timing/gimbal_history.hpp"

# include <algorithm>
# include <cmath>
# include <limits>

namespace kalfilter
{
namespace
{

constexpr double kTimeEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

//检查4元数的4个值不是极大极小值并且不是0元数
bool validQuaternion(const Eigen::Quaterniond& q)
{
    return q.coeffs().allFinite() && q.norm() > 1e-9;
}

//环绕函数
double wrapPi(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

//取最短角差
float interpolateAngle(float from, float to, double alpha)
{
    const double delta = wrapPi(
        static_cast<double>(to) - static_cast<double>(from));
    return static_cast<float>(
        wrapPi(static_cast<double>(from) + alpha * delta));
}

}   // namespace

//初始化列表，防止参数变零
GimbalFeedbackHistory::GimbalFeedbackHistory(
    double max_history_seconds,
    double max_interpolation_gap_seconds,
    double max_prediction_seconds)
    : max_history_seconds_(
          max_history_seconds > 0.0 ? max_history_seconds : 2.0),
      max_interpolation_gap_seconds_(
          max_interpolation_gap_seconds > 0.0
              ? max_interpolation_gap_seconds
              : 0.1),
      max_prediction_seconds_(
          max_prediction_seconds > 0.0 ? max_prediction_seconds : 0.05)
{
}

//输入
bool GimbalFeedbackHistory::push(const GimbalSample& sample)
{
    //检查有效性
    if (!std::isfinite(sample.host_time) ||
        !validQuaternion(sample.q) ||
        !std::isfinite(sample.pitch) ||
        !std::isfinite(sample.pitch_vel) ||
        !std::isfinite(sample.yaw) ||
        !std::isfinite(sample.yaw_vel) ||
        !std::isfinite(sample.bullet_speed))
    {
        return false;
    }

    //复制一份，四元数单位化
    GimbalSample normalized = sample;
    normalized.q.normalize();

    //加锁
    std::lock_guard<std::mutex> lock(mutex_);

    if (!samples_.empty())
    {
        const double dt = normalized.host_time - samples_.back().host_time;
        if (dt <= kTimeEpsilon) return false;

        // q 与 -q 表示同一个姿态，保持符号连续可避免 SLERP出错
        if (normalized.q.dot(samples_.back().q) < 0.0)
        {
            normalized.q.coeffs() *= -1.0;
        }
    }

    samples_.push_back(normalized);
    pruneUnlocked();
    return true;
}

void GimbalFeedbackHistory::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

bool GimbalFeedbackHistory::interpolate(
    double host_time,
    GimbalSample& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return interpolateUnlocked(host_time, out);
}

bool GimbalFeedbackHistory::latest(GimbalSample& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) return false;
    out = samples_.back();
    return true;
}

bool GimbalFeedbackHistory::predict(
    double host_time,
    GimbalSample& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return predictUnlocked(host_time, out);
}

bool GimbalFeedbackHistory::empty() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.empty();
}

std::size_t GimbalFeedbackHistory::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

bool GimbalFeedbackHistory::interpolateUnlocked(
    double host_time,
    GimbalSample& out) const
{
    if (!std::isfinite(host_time) || samples_.empty()) return false;

    //第一个 host_time >= t 的时间
    const auto next = std::lower_bound(
        samples_.begin(),
        samples_.end(),
        host_time,
        [](const GimbalSample& sample, double time)  //返回第一个 host_time >= t 的迭代器
        {
            return sample.host_time < time;
        });

    if (next == samples_.end()) return false;

    //视为精准命中，直接取值
    if (std::abs(next->host_time - host_time) <= kTimeEpsilon)
    {
        out = *next;
        return true;
    }

    if (next == samples_.begin()) return false;

    const auto previous = std::prev(next);
    const double sample_gap = next->host_time - previous->host_time;
    if ( sample_gap > max_interpolation_gap_seconds_)
    {
        return false;
    }

    const double alpha =
        (host_time - previous->host_time) / sample_gap;
    if (alpha < 0.0 || alpha > 1.0) return false;

    out.host_time = host_time;
    out.board_time_us = previous->board_time_us;
    out.q = previous->q.slerp(alpha, next->q).normalized();
    out.pitch = static_cast<float>(
        previous->pitch +
        alpha * (next->pitch - previous->pitch));
    out.pitch_vel = static_cast<float>(
        previous->pitch_vel +
        alpha * (next->pitch_vel - previous->pitch_vel));
    out.yaw = interpolateAngle(previous->yaw, next->yaw, alpha);
    out.yaw_vel = static_cast<float>(
        previous->yaw_vel +
        alpha * (next->yaw_vel - previous->yaw_vel));
    out.bullet_speed = static_cast<float>(
        previous->bullet_speed +
        alpha * (next->bullet_speed - previous->bullet_speed));

    // mode 不能做连续插值，保持左值直到下一个样本。
    out.mode = previous->mode;
    return true;
}

//限时外拓
bool GimbalFeedbackHistory::predictUnlocked(
    double host_time,
    GimbalSample& out) const
{
    if (!std::isfinite(host_time) || samples_.empty()) return false;

    const GimbalSample& newest = samples_.back();
    if (host_time <= newest.host_time + kTimeEpsilon)
    {
        return interpolateUnlocked(host_time, out);
    }

    const double horizon = host_time - newest.host_time;
    if (horizon > max_prediction_seconds_ || samples_.size() < 2)
    {
        return false;
    }

    const GimbalSample& previous = samples_[samples_.size() - 2];
    const double sample_dt = newest.host_time - previous.host_time;
    if (sample_dt <= 0.0 ||
        sample_dt > max_interpolation_gap_seconds_)
    {
        return false;
    }

    /*
     * q 表示 W <- C。q_prev^-1 * q_new 是相机自身的增量旋转，
     * 取旋转向量后视为相机角速度，再沿最后姿态短时积分。
     */
    const Eigen::Quaterniond delta =
        (previous.q.conjugate() * newest.q).normalized();
    const Eigen::AngleAxisd delta_axis(delta);
    const Eigen::Vector3d angular_velocity =
        delta_axis.axis() * (delta_axis.angle() / sample_dt);
    const double horizon_angle = angular_velocity.norm() * horizon;
    Eigen::Quaterniond horizon_delta = Eigen::Quaterniond::Identity();
    if (horizon_angle > 1e-12)
    {
        horizon_delta = Eigen::Quaterniond(Eigen::AngleAxisd(
            horizon_angle, angular_velocity.normalized()));
    }

    out = newest;
    out.host_time = host_time;
    out.q = (newest.q * horizon_delta).normalized();
    out.pitch = static_cast<float>(
        newest.pitch + newest.pitch_vel * horizon);
    out.yaw = static_cast<float>(wrapPi(
        static_cast<double>(newest.yaw) +
        static_cast<double>(newest.yaw_vel) * horizon));
    return true;
}

//清理过久的历史
void GimbalFeedbackHistory::pruneUnlocked()
{
    if (samples_.empty()) return;

    const double oldest_allowed =
        samples_.back().host_time - max_history_seconds_;
    while (samples_.size() > 1 &&
           samples_.front().host_time < oldest_allowed)
    {
        samples_.pop_front();
    }
}

}   // namespace kalfilter
