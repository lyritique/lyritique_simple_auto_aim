#ifndef AUTOAIM_GIM_HISTORY_
#define AUTOAIM_GIM_HISTORY_

#include <cstddef>
#include <cstdint>
#include <deque>
#include <eigen3/Eigen/Geometry>
#include <mutex>

namespace kalfilter {

/*
 * 一条 C 板反馈。host_time 使用上位机的 steady_clock 秒数；
 * q 已经由 C 板换算成“相机系 -> 参考世界系”，上位机不再乘安装矩阵。
 */
struct GimbalSample {
  // 作用：这条反馈在“我的时间轴”（上位机 steady_clock）上的时刻。实现：到达时已经换算好，历史排序全靠它。
  double host_time{};
  // 作用：板上原始时间戳，留着追溯和调试。实现：原样搬运，不参与排序计算。
  uint64_t board_time_us{};

  // 作用：云台姿态，相机系到参考世界系，插值的主角。实现：Eigen 四元数，默认单位旋转，绝不留空指针式姿势。
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};

  // 作用：俯仰角，只给显示和日志凑热闹。实现：float 存，插值时线性。
  float pitch{};
  // 作用：俯仰角速度，预测外推时当速度用。实现：float 存，单位弧度每秒。
  float pitch_vel{};
  // 作用：偏航角，会绕圈，插值要特殊照顾。实现：float 存，插值走 wrapPi 最短路径。
  float yaw{};
  // 作用：偏航角速度，预测时直接乘时间。实现：float 存。
  float yaw_vel{};
  // 作用：当前弹速，解算提前量要用。实现：float 存，插值时线性过渡。
  float bullet_speed{};

  // 作用：云台工作模式，离散量，不能插值。实现：uint8 存，插值时直接抄左端点。
  uint8_t mode{};
};

/*
 * 按时间维护最近的云台反馈。
 *
 * interpolate() 只接受被两个历史样本夹住的时间，绝不为了图像对齐而外推。
 * predict() 只用于未来控制预测，并且限制外推时长。
 */
class GimbalFeedbackHistory {
 public:
  // 作用：给历史仓库定容量和脾气。实现：三个时长参数都做了“非正就取默认”的兜底，避免零窗口把逻辑逼疯。
  explicit GimbalFeedbackHistory(double max_history_seconds = 2.0, double max_interpolation_gap_seconds = 0.1,
                                 double max_prediction_seconds = 0.05);

  // 时间必须严格递增；重复或乱序样本返回 false。
  // 作用：存一条反馈，顺序不对的直接退货。实现：验数、单位化、锁内比时间、修四元数符号，然后入队并裁剪。
  bool push(const GimbalSample& sample);
  // 作用：把仓库清空，重连或切模式时用。实现：锁内 clear 掉 deque，一步到位。
  void clear();

  // 在 [a.host_time, b.host_time] 内 SLERP/线性插值；区间外返回 false。
  // 作用：给定时刻问姿态，必须夹在两个样本中间。实现：套锁后交给 interpolateUnlocked，二分找邻居再插值。
  bool interpolate(double host_time, GimbalSample& out) const;

  // 返回当前历史中的最新有效样本。
  // 作用：拿到“刚刚那一帧”的云台状态。实现：锁内取 deque 尾巴，空仓库就返回 false。
  bool latest(GimbalSample& out) const;

  // 历史区间内调用 interpolate；未来最多外推 max_prediction_seconds。
  // 作用：既能查历史，也能把状态往后推一小段给控制用。实现：套锁后走 predictUnlocked，历史区间内退化成插值。
  bool predict(double host_time, GimbalSample& out) const;

  // 作用：问一句“仓库空不空”。实现：锁内转调 deque::empty。
  bool empty() const;
  // 作用：报一下存货条数，调试用。实现：锁内返回 deque::size。
  std::size_t size() const;

 private:
  // 作用：真正干插值的活，调用方保证锁已加。实现：lower_bound 找右邻居，左右两端线性/SLERP 混合。
  bool interpolateUnlocked(double host_time, GimbalSample& out) const;
  // 作用：真正干预测的活，调用方保证锁已加。实现：历史段转插值，未来段用末两帧估角速度短时积分。
  bool predictUnlocked(double host_time, GimbalSample& out) const;
  // 作用：按时间淘汰老样本，防止内存随比赛时长一起膨胀。实现：以最新样本为基准算截止线，从队头一路 pop。
  void pruneUnlocked();

  // 作用：仓库保留多久的历史（秒），决定内存和搜索成本。实现：构造时兜底，pruneUnlocked 按它划线。
  double max_history_seconds_{};
  // 作用：两样本间隔超过它就认为数据断档，拒绝插值。实现：插值时比较 sample_gap，预测时也拿它把关。
  double max_interpolation_gap_seconds_{};
  // 作用：未来外推的最大允许时长，防止把云台吹到外太空。实现：predict 里用 horizon 和它比大小。
  double max_prediction_seconds_{};

  // 作用：保护 samples_，因为接收线程写、算法线程读。实现：所有 public 方法进门先 lock_guard。
  mutable std::mutex mutex_;
  // 双头容器
  //  作用：按时间排好的样本队列，头部最老、尾部最新。实现：std::deque 支持两端 O(1) 增删，正好匹配滑窗需求。
  std::deque<GimbalSample> samples_;
};

}  // namespace kalfilter

#endif
