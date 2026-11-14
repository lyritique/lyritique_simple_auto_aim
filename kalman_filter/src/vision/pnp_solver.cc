#include "vision/pnp_solver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace pnp {

namespace {

constexpr double kPi{3.14159265358979323846};

struct candidate {
  double rms{};
  double cost{};
  double tz{};
  double yaw{};
  Eigen::Matrix<double, 4, 1> measurement{};
  Eigen::Vector3d normal{Eigen::Vector3d::Zero()};
  cv::Mat rvec{};
  cv::Mat tvec{};
  int branch{};
  bool inward{};
  bool reverse{};
};

double wrapPi(double a)  // 我怎么觉得好像写过
{
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

}  // namespace

std::array<cv::Point2f, 4> pnpSolver::canonicalize(const std::array<cv::Point2d, 4>& raw, bool reverse) const {
  // 构造中心
  cv::Point2d centre{0.0f, 0.0f};
  for (const auto& p : raw) centre += p;
  centre *= 0.25f;

  // 用order记顺序，再排序
  //
  std::array<int, 4> order{0, 1, 2, 3};
  std::sort(order.begin(), order.end(), [&](int a, int b) {
    const double aa = std::atan2(raw[a].y - centre.y, raw[a].x - centre.x);
    const double bb = std::atan2(raw[b].y - centre.y, raw[b].x - centre.x);
    return aa < bb;
  });

  std::array<cv::Point2f, 4> ring{};
  for (int i = 0; i < 4; ++i) ring[i] = raw[order[i]];

  // 反向遍历 = 换绕向，用来消掉镜像解
  if (reverse) std::swap(ring[1], ring[3]);

  // 旋转环，让 ring[0] -> ring[1] 落在最长边（135mm 那条）上
  int longest = 0;
  float best = -1.0f;
  for (int i = 0; i < 4; ++i) {
    const cv::Point2f d = ring[(i + 1) % 4] - ring[i];
    const float len = d.x * d.x + d.y * d.y;
    if (len > best) {
      best = len;
      longest = i;
    }
  }

  std::array<cv::Point2f, 4> out{};
  for (int i = 0; i < 4; ++i) out[i] = ring[(longest + i) % 4];
  return out;
}  // namespace pnp

// 作用：用配置里的内参构造相机矩阵和畸变系数。实现：K 按 fx/fy/cx/cy 填成 3x3，D 从 vector 原样搬成 1xN。
pnpSolver::pnpSolver(const core::IntrinsicsConfig& intrinsics) {
  // 作用：拼出 3x3 内参矩阵，畸变模型只认这个对角线结构。实现：cv::Mat_ 逐项写入，末行固定 0 0 1。
  camera_matrix_ =
      (cv::Mat_<double>(3, 3) << intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0);

  // 作用：把任意长度的畸变数组搬进 OpenCV 的 1xN 矩阵，4/5/8 系数都能过。实现：vector 区间构造后 clone 一份独立存储。
  std::vector<double> d(intrinsics.dist.begin(), intrinsics.dist.end());
  distortion_coeffs_ = cv::Mat(d, true).clone();
}

void pnpSolver::input(const std::array<cv::Point2f, 4>& image_corners, bool big) {
  // 把imagecorner赋值给corners
  for (std::size_t i = 0; i < corners.size(); ++i) corners[i] = cv::Point2d(image_corners[i]);

  armorPoint = big ? config.big_armorPoint : config.small_armorPoint;

  output.setZero();
  output_valid_ = false;
  proj_err = std::numeric_limits<double>::infinity();
}

// 先直接cv的Solver
void pnpSolver::easySolver() {
  output.setZero();
  output_valid_ = false;
  proj_err = std::numeric_limits<double>::infinity();
  last_branch_ = -1;

  if (corners.size() != 4) return;

  std::vector<candidate> candidates;

  for (const bool reverse : {false, true}) {
    const std::array<cv::Point2f, 4> ordered = canonicalize(corners, reverse);

    std::vector<cv::Mat> rvecs{}, tvecs{};
    const int nsol = cv::solvePnPGeneric(armorPoint, ordered, camera_matrix_, distortion_coeffs_, rvecs, tvecs, false,
                                         cv::SOLVEPNP_IPPE, cv::noArray());
    if (nsol <= 0) continue;

    for (int i = 0; i < nsol; ++i) {
      const double tz = tvecs[i].at<double>(2);
      if (!(tz > 0.0)) continue;

      cv::Mat R;
      cv::Rodrigues(rvecs[i], R);

      std::vector<cv::Point2d> reproj{};
      cv::projectPoints(armorPoint, rvecs[i], tvecs[i], camera_matrix_, distortion_coeffs_, reproj);
      if (reproj.size() != 4) continue;

      double singleErr{};
      for (int k = 0; k < 4; ++k) {
        const cv::Point2d diff = reproj[k] - cv::Point2d(ordered[k]);
        singleErr += diff.dot(diff);
      }
      const double rms = std::sqrt(singleErr / 4.0);

      if (!std::isfinite(rms) || rms > config.max_reproj_err) continue;

      // 板法向是板坐标系的 x 轴
      const Eigen::Vector3d normal(R.at<double>(0, 0), R.at<double>(1, 0), R.at<double>(2, 0));
      const Eigen::Vector3d tvec_e(tvecs[i].at<double>(0), tvecs[i].at<double>(1), tvecs[i].at<double>(2));
      const double yaw = std::atan2(normal.x(), normal.z());

      // 毫无用处，原因见class定义处
      double dyaw = 0.0;
      if (std::isfinite(previous_yaw)) dyaw = wrapPi(yaw - previous_yaw);

      candidate c;
      c.rms = rms;
      c.cost = rms + config.kyaw * dyaw * dyaw;
      c.tz = tz;
      c.yaw = yaw;
      c.normal = normal;
      c.rvec = rvecs[i];
      c.tvec = tvecs[i];
      c.branch = i;
      c.inward = normal.dot(tvec_e) > 0.0;
      c.reverse = reverse;
      c.measurement << tvec_e.x() / 1000.0, tvec_e.y() / 1000.0, tvec_e.z() / 1000.0, yaw;
      candidates.push_back(c);
    }
  }

  if (candidates.empty()) return;

  // 选解顺序：法向必须朝向相机（拍号那一面），再比 重投影误差 + yaw 连续性
  auto better = [](const candidate& a, const candidate& b) {
    if (a.inward != b.inward) return !a.inward;
    return a.cost < b.cost;
  };
  const candidate chosen = *std::min_element(candidates.begin(), candidates.end(), better);

  std::array<cv::Point2f, 4> ordered_chosen = canonicalize(corners, chosen.reverse);

  // 用 LM 精修；只有误差下降且 yaw 没跳变才采纳（避免 LM 跳到另一个分支）
  cv::Mat rvec_out = chosen.rvec.clone();
  cv::Mat tvec_out = chosen.tvec.clone();
  double rms_out = chosen.rms;
  try {
    cv::Mat rvec_lm = chosen.rvec.clone();
    cv::Mat tvec_lm = chosen.tvec.clone();
    cv::solvePnPRefineLM(armorPoint, ordered_chosen, camera_matrix_, distortion_coeffs_, rvec_lm, tvec_lm);

    std::vector<cv::Point2d> reproj{};
    cv::projectPoints(armorPoint, rvec_lm, tvec_lm, camera_matrix_, distortion_coeffs_, reproj);
    double sq = 0.0;
    for (int k = 0; k < 4; ++k) {
      const cv::Point2d diff = reproj[k] - cv::Point2d(ordered_chosen[k]);
      sq += diff.dot(diff);
    }
    const double rms_lm = std::sqrt(sq / 4.0);

    cv::Mat R_lm;
    cv::Rodrigues(rvec_lm, R_lm);
    const double yaw_lm = std::atan2(R_lm.at<double>(0, 0), R_lm.at<double>(2, 0));
    const double dyaw_lm = std::fabs(wrapPi(yaw_lm - chosen.yaw));

    if (std::isfinite(rms_lm) && rms_lm + 0.02 < chosen.rms && dyaw_lm < 0.30 && tvec_lm.at<double>(2) > 0.0) {
      rvec_out = rvec_lm;
      tvec_out = tvec_lm;
      rms_out = rms_lm;
    }
  } catch (const cv::Exception&) {
    // 精修失败就保留 IPPE 的解
  }

  cv::Mat R_final;
  cv::Rodrigues(rvec_out, R_final);
  const Eigen::Vector3d normal_final(R_final.at<double>(0, 0), R_final.at<double>(1, 0), R_final.at<double>(2, 0));
  const double yaw_final = std::atan2(normal_final.x(), normal_final.z());

  // solvePnP 的平移与对象点同单位（毫米）
  output << tvec_out.at<double>(0) / 1000.0, tvec_out.at<double>(1) / 1000.0, tvec_out.at<double>(2) / 1000.0,
      yaw_final;
  proj_err = rms_out;
  output_valid_ = true;

  // 不可信的无用数据，原因见class定义处
  previous_yaw = yaw_final;

  previous_normal = normal_final;
  last_normal_ = normal_final;
  last_branch_ = chosen.branch;
  last_big_ = std::abs(armorPoint.front().z - config.big_armorPoint.front().z) < 1e-6;
}

// 并无用处，原因见class定义处
void pnpSolver::renewPrevious(double& best_yaw) { previous_yaw = best_yaw; }

PnPResult pnpSolver::getOutput() {
  PnPResult result;
  result.valid = output_valid_;
  result.measurement = output;
  result.err = proj_err;
  result.normal = last_normal_;
  return result;
}
}  // namespace pnp
