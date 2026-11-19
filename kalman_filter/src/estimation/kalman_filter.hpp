#ifndef AUTOAIM_ESTIMATION_KALMAN_FILTER_HPP
#define AUTOAIM_ESTIMATION_KALMAN_FILTER_HPP

#include <eigen3/Eigen/Eigen>
#include <limits>
#include <vector>

namespace kalfilter {

struct KalmanConfig {
  // Q
  // 车心/角速度的加速度噪声。原来 100/400 相当于允许 ~10 m/s²、20 rad/s² 的随机加速度，
  // 单板看旋转目标时位置和 yaw 会互相抢：滤波器宁愿平移车心也不转 yaw。
  // 10/50 仍能跟住机动，但把车心钉住得多。
  double p_coord = 20;
  double p_yaw = 225;
  double p_r{1e-6};
  // Δr/Δz 只在 id 奇偶切换时才被约束，可观测性弱，噪声给得远小于位置速度，
  // 相当于强先验：允许缓慢修正，不许乱跑。
  double p_dr{1e-7};
  double p_dz{1e-7};

  // R
  // yac对角噪声
  double ele{0.063};
  double azi{0.063};
  double k_dist{0.05};
  double yaw{0.1};
  // 板子斜视时法向方位角的误差按 1/cos(斜视角) 放大，这是放大后的上限。
  // 10 rad/s 目标的大部分时间都只有一块快立起来的板在画面里，它的 yaw 基本是随机数；
  // 但也不能放大到无穷，位置那三行还要靠它把车心钉住。
  double yaw_max{0.6};

  double assoc_pos_gate{2.5};
  double max_yaw_innovation{0.7853981633974483};  // 45 度
  double chi2_gate{9.49};

  // 狠狠限制r的协方差（5 mm，够吸收两对板半径差被 Δr 分走之后的余量）
  double kRadiusSigma = 5e-3;

  // 初始化不确定度（原来用 P.setIdentity() 当协方差，位置方差 1 m^2 太离谱）
  double init_sigma_pos{0.05};  // m
  double init_sigma_vel{4.0};   // m/s
  double init_sigma_yaw{0.25};  // rad
  double init_sigma_vyaw{3.0};  // rad/s

  // 两对装甲板的结构差异。Δr 是奇数对相对偶数对的半径差，
  // Δz 是同一对相对偶数对的高度偏移。给零方差会让它们永远学不动。
  double init_dr{0.0};         // m
  double init_dz{0.05};        // m
  double init_sigma_dr{0.01};  // m
  double init_sigma_dz{0.03};  // m

  // 多久没有新观测就让状态失效。纯靠预测外推越久越离谱，云台会照着空想的角度转。
  double lost_timeout{0.3};  // s
};

// 相机系球坐标，用于求R观测噪声协方差矩阵，噪声天然的坐标系是 YPD
struct cam_ypd {
  double azi{0.063};
  double ele{0.063};
  double dist{};
};

struct ObsAssociation {
  int id = -1;  // 匹配到的 id
  double d2 = std::numeric_limits<double>::infinity();
  bool valid = false;                // 是否通过门限
  Eigen::Matrix<double, 4, 1> nu{};  // 残差（yaw 已 wrap）
  Eigen::Matrix<double, 4, 11> H{};  // 雅可比
  Eigen::Matrix4d Sinv{};            // S 的逆，供更新复用
};

struct armerState {
  Eigen::Matrix<double, 4, 1> measurement;
  kalfilter::ObsAssociation testResult;
};

struct storedID {
  int id{};  // 0~3
  bool isThisFrame = false;
};

struct KFResult {
  // px, py(高度), pz, yaw, vx, vy, vz, vyaw, r, Δr, Δz
  Eigen::Matrix<double, 11, 1> state = Eigen::Matrix<double, 11, 1>::Zero();
  bool valid = false;
};

class KalmanFilter {
 public:
  explicit KalmanFilter(const KalmanConfig& config = KalmanConfig{});

  void init(armerState& measurement);
  void reset();
  void predict(const double& dt);
  // R_C0C 把观测噪声从相机系转到世界系，必须和 measurement 用同一个旋转。
  // cos_obliquity = |板法向 · 指向相机的视线|，正对为 1；只用来放大 yaw 那一行的噪声。
  void update(armerState& armor, const cam_ypd& ypd, const Eigen::Matrix3d& R_C0C, double cos_obliquity = 1.0);
  // 这个是每一帧的结束处理id_list的
  void renewArmorList();
  KFResult getOutput() const;

 private:
  // X=(px,py,pz,yaw,vx,vy,vz,vyaw,r,Δr,Δz)
  // 前 9 维是车体本身，后两维是两对装甲板的结构差异，只作用在奇数 id 上。
  Eigen::Matrix<double, 11, 1> X{};
  Eigen::Matrix<double, 11, 11> P{};
  Eigen::Matrix<double, 11, 11> F{};

  bool initialized = false;

  // 距上一次真正吃到观测过了多久，丢目标判定用。
  double time_since_update_{};

  KalmanConfig config{};

  std::vector<storedID> id_list{};
  int max_id{};
};

}  // namespace kalfilter

#endif
