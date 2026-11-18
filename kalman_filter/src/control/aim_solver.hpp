#ifndef AUTOAIM_CONTROL_AIM_SOLVER_HPP
#define AUTOAIM_CONTROL_AIM_SOLVER_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "estimation/kalman_filter.hpp"

namespace control {

struct AimConfig {
  // C 板没报弹速时的兜底值，m/s。
  double bullet_speed{25.0};
  // C 板世界系里 yaw / pitch 的正方向，实测反了就传 -1。
  // 先用 --aim-debug 打印的角度和 C 板自报值对比确定。
  double yaw_sign{1.0};
  double pitch_sign{1.0};
  // 零位补偿，弧度。实测确认过没有固定偏差，一般留 0。
  double yaw_offset{0.0};
  double pitch_offset{0.0};
  // 相机系（x 右/y 下/z 前）-> C 板 IMU 体轴（x 前/y 左/z 上）。
  // C 板四元数描述的是体轴，而 EKF 的观测向量是相机系，不换会差一个 90°。
  bool cam_remap{true};
  // C 板世界系哪根轴朝天：false = y 轴（x-z 水平），true = z 轴（x-y 水平）。
  bool z_up{true};
  // 弹道下坠补偿。未标定前默认关，只打纯几何方向。
  bool compensate_drop{false};
  double gravity{9.8};
};

// 一条解算结果，角度都在 C 板世界系里。
struct AimSolution {
  bool valid{};
  // 预测时刻的目标点，算法世界 C0。
  Eigen::Vector3d point_c0{};
  // 从相机指向目标点的单位向量，C 板世界系。
  Eigen::Vector3d dir_world{};
  double yaw{};
  double pitch{};
  double distance{};
  double flight_time{};
  int armor_id{-1};

  // 给 C 板当前馈用的角速度/角加速度，由相邻两帧指令差分并低通得到。
  double yaw_vel{};
  double pitch_vel{};
  double yaw_acc{};
  double pitch_acc{};
};

/*
 * 把 EKF 状态换成一条云台指令。
 *
 * 1. 按 latency + 飞行时间把状态外推到命中时刻（飞行时间和距离互相耦合，迭代两次）；
 * 2. 在预测时刻挑出正对相机的那块装甲板，取它的中心作为瞄准点；
 * 3. 用 q_init（相机系 -> C 板世界系）把 C0 下的方向转回 C 板世界系。
 *
 * 第 3 步是关键：算法世界 C0 是"首帧相机系"，相机耷拉着时它也跟着歪；
 * 而 q_init 本身就带着那个耷拉角，乘回去后发出去的角度就是 C 板世界系里
 * 相对水平的量，不需要额外标定零位。
 */
class AimSolver {
 public:
  explicit AimSolver(const AimConfig& config = {});

  // 参数：state —— EKF 状态（C0）；q_init —— 首帧相机姿态，相机系 -> C 板世界系；
  //      latency_seconds —— 从曝光到指令生效的延迟（不含飞行时间）；
  //      bullet_speed —— C 板报的弹速，<=0 用配置兜底；
  //      now_seconds —— 上位机单调时钟，只用于差分出角速度。
  // 返回：解算结果；state 无效时 valid=false。
  // q_init / q_now —— 首帧和当前帧的相机姿态（相机系 -> C 板世界系）。
  // c0_to_world —— 非空时表示 EKF 状态已经在水平世界 C0 里，直接用这个矩阵转到 C 板世界系，
  //                不再走 q_init/q_now 的相机系换算（对应有 IMU 的实时模式）。
  AimSolution solve(const kalfilter::KFResult& state, const Eigen::Quaterniond& q_init,
                    const Eigen::Quaterniond& q_now, double latency_seconds, double bullet_speed, double now_seconds,
                    const Eigen::Matrix3d* c0_to_world = nullptr);

  // 换目标或重连时清掉差分历史，免得拿上一条指令算出一个假的角速度。
  void reset();
  const AimConfig& config() const noexcept { return config_; }

 private:
  AimConfig config_;

  bool has_last_{};
  double last_time_{};
  double last_yaw_{};
  double last_pitch_{};
  double last_yaw_vel_{};
  double last_pitch_vel_{};
};

}  // namespace control

#endif
