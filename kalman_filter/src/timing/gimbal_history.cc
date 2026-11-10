# include "timing/gimbal_history.hpp"

# include <algorithm>
# include <cmath>
# include <limits>

namespace kalfilter
{
namespace
{

// 作用：判断两个时间算不算“同一个时刻”的容差。实现：1e-9 秒，比调度抖动小、比浮点误差大。
constexpr double kTimeEpsilon = 1e-9;
// 作用：圆周率，角度绕圈全靠它。实现：手写常量，省得为这一个数去链 libm 的 M_PI。
constexpr double kPi = 3.14159265358979323846;

//检查4元数的4个值不是极大极小值并且不是0元数
// 作用：给四元数做体检，挡住 NaN、inf 和全零。实现：coeffs() 全有限且模长大于 1e-9 才算合格。
bool validQuaternion(const Eigen::Quaterniond& q)
{
    return q.coeffs().allFinite() && q.norm() > 1e-9;
}

//环绕函数
// 作用：把任意角度塞回 (-pi, pi]，避免插值绕远路。实现：大于 pi 就减整圈，小于 -pi 就加整圈，while 到收敛。
double wrapPi(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

//取最短角差
// 作用：在圆周上从 from 走到 to 的插值，保证走的是近路。实现：先 wrapPi 求最短差值，按 alpha 缩放后加到起点再绕回。
float interpolateAngle(float from, float to, double alpha)
{
    const double delta = wrapPi(
        static_cast<double>(to) - static_cast<double>(from));
    return static_cast<float>(
        wrapPi(static_cast<double>(from) + alpha * delta));
}

}   // namespace

//初始化列表，防止参数变零
// 作用：把三个时长参数存好，顺手把非法值换成默认值。实现：三目运算符逐个兜底，非正数一律回落到默认。
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
// 作用：收一条云台反馈，过五关斩六将后才入库。实现：验数、单位化、时间递增检查、四元数符号对齐、入队裁剪。
bool GimbalFeedbackHistory::push(const GimbalSample& sample)
{
    //检查有效性
    // 作用：把 NaN、inf、零四元数这种“毒样本”挡在门外。实现：逐个字段 isfinite 加四元数体检，任一不过就退货。
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
    // 作用：不碰调用方的数据，也让后续 SLERP 拿到干净的旋转。实现：拷贝一份样本，再对 q 做 normalize。
    GimbalSample normalized = sample;
    normalized.q.normalize();

    //加锁
    // 作用：写队列前先占坑，防止读线程看到半截状态。实现：lock_guard 守到函数结束。
    std::lock_guard<std::mutex> lock(mutex_);

    // 作用：时间必须严格往前走，否则历史排序就乱套了。实现：和队尾比 dt，小于等于 1e-9 一律拒绝。
    if (!samples_.empty())
    {
        const double dt = normalized.host_time - samples_.back().host_time;
        if (dt <= kTimeEpsilon) return false;

        // q 与 -q 表示同一个姿态，保持符号连续可避免 SLERP出错
        // 作用：把四元数翻到和上一帧同一半球，防止 SLERP 走 360 度大回环。实现：点积为负就整体乘 -1，等价旋转不变。
        if (normalized.q.dot(samples_.back().q) < 0.0)
        {
            normalized.q.coeffs() *= -1.0;
        }
    }

    // 作用：正式入库，并顺手清掉过老的历史。实现：push_back 到队尾，再调 pruneUnlocked 从队头裁。
    samples_.push_back(normalized);
    pruneUnlocked();
    return true;
}

// 作用：一键清仓。实现：锁内 clear，deque 全丢。
void GimbalFeedbackHistory::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

// 作用：对外提供带锁的插值入口。实现：拿锁后转调内部无锁版本，锁的边界清清楚楚。
bool GimbalFeedbackHistory::interpolate(
    double host_time,
    GimbalSample& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return interpolateUnlocked(host_time, out);
}

// 作用：取最新样本，给“现在怎么样”这类查询用。实现：锁内看队列空不空，不空就抄队尾。
bool GimbalFeedbackHistory::latest(GimbalSample& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) return false;
    out = samples_.back();
    return true;
}

// 作用：对外提供带锁的预测入口。实现：拿锁后转调 predictUnlocked。
bool GimbalFeedbackHistory::predict(
    double host_time,
    GimbalSample& out) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return predictUnlocked(host_time, out);
}

// 作用：查库存是否为空。实现：锁内转调 deque::empty。
bool GimbalFeedbackHistory::empty() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.empty();
}

// 作用：报库存条数。实现：锁内转调 deque::size。
std::size_t GimbalFeedbackHistory::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

// 作用：在两帧之间还原任意时刻的云台状态。实现：二分找时间右邻居，用 alpha 对四元数 SLERP、对其余量线性混合。
bool GimbalFeedbackHistory::interpolateUnlocked(
    double host_time,
    GimbalSample& out) const
{
    // 作用：时间和库存任一不可用就别算。实现：isfinite 加 empty 一道挡下。
    if (!std::isfinite(host_time) || samples_.empty()) return false;

    //第一个 host_time >= t 的时间
    // 作用：找到时间轴上第一个不比目标早的样本，作为插值右端点。实现：lower_bound 加自定义比较，历史已有序所以是 O(log n)。
    const auto next = std::lower_bound(
        samples_.begin(),
        samples_.end(),
        host_time,
        [](const GimbalSample& sample, double time)  //返回第一个 host_time >= t 的迭代器
        {
            return sample.host_time < time;
        });

    // 作用：目标比所有样本都新，历史里没人接得住。实现：命中 end 直接返回 false，绝不外推。
    if (next == samples_.end()) return false;

    //视为精准命中，直接取值
    // 作用：时间正好落在样本上，省掉插值误差。实现：差值在容差内就直接拷贝这一帧。
    if (std::abs(next->host_time - host_time) <= kTimeEpsilon)
    {
        out = *next;
        return true;
    }

    // 作用：目标比第一帧还早，左端点不存在。实现：next 是 begin 就无解。
    if (next == samples_.begin()) return false;

    // 作用：拿到左端点，组成插值区间。实现：prev 往前挪一格。
    const auto previous = std::prev(next);
    // 作用：算两帧间隔，顺便判断数据有没有断档。实现：右减左得秒数。
    const double sample_gap = next->host_time - previous->host_time;
    // 作用：间隔太大说明中间漏包，硬插等于编数据。实现：超过 max_interpolation_gap_seconds_ 就拒绝。
    if ( sample_gap > max_interpolation_gap_seconds_)
    {
        return false;
    }

    // 作用：算目标在区间里的比例，0 是左端、1 是右端。实现：线性归一化。
    const double alpha =
        (host_time - previous->host_time) / sample_gap;
    // 作用：双保险，比例跑出 [0,1] 说明前面算错了。实现：越界直接返回 false。
    if (alpha < 0.0 || alpha > 1.0) return false;

    // 作用：时间直接用查询值，不必插。实现：原样赋值。
    out.host_time = host_time;
    // 作用：板卡时间没法插值，先抄左端点，仅作追溯。实现：原样搬运 previous 的值。
    out.board_time_us = previous->board_time_us;
    // 作用：姿态走球面最短路径插值，再单位化防数值漂移。实现：Eigen 的 slerp 加 normalize。
    out.q = previous->q.slerp(alpha, next->q).normalized();
    // 作用：俯仰角线性过渡。实现：左值加 alpha 乘差值。
    out.pitch = static_cast<float>(
        previous->pitch +
        alpha * (next->pitch - previous->pitch));
    // 作用：俯仰角速度线性过渡。实现：同上，左值加 alpha 乘差。
    out.pitch_vel = static_cast<float>(
        previous->pitch_vel +
        alpha * (next->pitch_vel - previous->pitch_vel));
    // 作用：偏航角要绕圈插，否则会从 +179 度一路扫到 -179 度。实现：交给 interpolateAngle 走最短角差。
    out.yaw = interpolateAngle(previous->yaw, next->yaw, alpha);
    // 作用：偏航角速度线性过渡。实现：左值加 alpha 乘差。
    out.yaw_vel = static_cast<float>(
        previous->yaw_vel +
        alpha * (next->yaw_vel - previous->yaw_vel));
    // 作用：弹速线性过渡，避免提前量跳变。实现：左值加 alpha 乘差。
    out.bullet_speed = static_cast<float>(
        previous->bullet_speed +
        alpha * (next->bullet_speed - previous->bullet_speed));

    // mode 不能做连续插值，保持左值直到下一个样本。
    // 作用：离散模式不插值，保持区间内一致。实现：直接抄 previous->mode。
    out.mode = previous->mode;
    return true;
}

//限时外拓
// 作用：把云台状态往未来推一小步，给控制提前量用。实现：历史区间内退化成插值，未来区间用末两帧估角速度再短时积分。
bool GimbalFeedbackHistory::predictUnlocked(
    double host_time,
    GimbalSample& out) const
{
    // 作用：时间和库存先过一遍。实现：isfinite + empty 双重门禁。
    if (!std::isfinite(host_time) || samples_.empty()) return false;

    // 作用：拿最新一帧当外推起点。实现：直接引用队尾，只读不拷贝。
    const GimbalSample& newest = samples_.back();
    // 作用：目标不在未来，预测这活儿不归我，交给插值。实现：时间不晚于最新帧（含容差）就转发 interpolateUnlocked。
    if (host_time <= newest.host_time + kTimeEpsilon)
    {
        return interpolateUnlocked(host_time, out);
    }

    // 作用：算要往前推多久。实现：目标减最新帧时间。
    const double horizon = host_time - newest.host_time;
    // 作用：外推太远或样本不足两条就没法估速度，直接认输。实现：horizon 超上限或 size<2 就返回 false。
    if (horizon > max_prediction_seconds_ || samples_.size() < 2)
    {
        return false;
    }

    // 作用：取倒数第二帧，和最新帧组队估角速度。实现：下标 size-2 直接访问。
    const GimbalSample& previous = samples_[samples_.size() - 2];
    // 作用：算两帧间隔并检查是否可用。实现：非正或超断档阈值都拒绝，免得除以垃圾。
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
    // 作用：求两帧之间的增量旋转。实现：前帧共轭乘后帧，得到相机自身坐标系下的变化，结果单位化防漂。
    const Eigen::Quaterniond delta =
        (previous.q.conjugate() * newest.q).normalized();
    // 作用：把增量旋转拆成“转轴 + 转角”。实现：AngleAxis 提取轴角表示。
    const Eigen::AngleAxisd delta_axis(delta);
    // 作用：把轴角除以时间，得到角速度向量（弧度每秒）。实现：单位轴乘上角速度大小。
    const Eigen::Vector3d angular_velocity =
        delta_axis.axis() * (delta_axis.angle() / sample_dt);
    // 作用：算这段时间总共要转多少弧度。实现：角速度模长乘 horizon。
    const double horizon_angle = angular_velocity.norm() * horizon;
    // 作用：给外推增量一个默认“原地不动”。实现：先赋单位四元数。
    Eigen::Quaterniond horizon_delta = Eigen::Quaterniond::Identity();
    // 作用：角速度确实转过东西时才构造旋转，避免零向量归一化。实现：角度大于 1e-12 才用归一化轴建四元数。
    if (horizon_angle > 1e-12)
    {
        horizon_delta = Eigen::Quaterniond(Eigen::AngleAxisd(
            horizon_angle, angular_velocity.normalized()));
    }

    // 作用：以外推起点为底稿，先把速度等字段原样继承。实现：整条样本拷贝到输出。
    out = newest;
    // 作用：把时间改到目标时刻，货物才发得出去。实现：直接覆盖 host_time。
    out.host_time = host_time;
    // 作用：姿态沿角速度转过去。实现：当前姿态右乘外推增量，再单位化。
    out.q = (newest.q * horizon_delta).normalized();
    // 作用：俯仰角按角速度线性外推。实现：当前角加角速度乘 horizon。
    out.pitch = static_cast<float>(
        newest.pitch + newest.pitch_vel * horizon);
    // 作用：偏航角同样外推，但必须绕回 (-pi, pi]。实现：加完后交给 wrapPi。
    out.yaw = static_cast<float>(wrapPi(
        static_cast<double>(newest.yaw) +
        static_cast<double>(newest.yaw_vel) * horizon));
    return true;
}

//清理过久的历史
// 作用：淘汰过期样本，控制内存和二分成本。实现：以最新时间为基准算截止线，从队头开始 pop，至少留一帧。
void GimbalFeedbackHistory::pruneUnlocked()
{
    // 作用：空仓库没什么可裁的。实现：empty 早退。
    if (samples_.empty()) return;

    // 作用：算出还允许保留的最老时刻。实现：最新时间减去历史上限。
    const double oldest_allowed =
        samples_.back().host_time - max_history_seconds_;
    // 作用：从队头淘汰太老的数据，但保住最后一帧当底线。实现：size>1 且 front 早于截止线就 pop_front，循环到合规矩。
    while (samples_.size() > 1 &&
           samples_.front().host_time < oldest_allowed)
    {
        samples_.pop_front();
    }
}

}   // namespace kalfilter
