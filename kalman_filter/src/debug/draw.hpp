#ifndef AUTOAIM_DRAW_
#define AUTOAIM_DRAW_

#include <opencv2/core.hpp>

#include "estimation/kalman_filter.hpp"
#include "vision/armor_detector.hpp"

namespace draw {

void drawDetections(cv::Mat& frame, const detector::ArmorDetections& detections);

void drawStateText(cv::Mat& frame, const kalfilter::KFResult& state);

// R_C0C 把算法世界系转到相机系（取转置用），状态来自世界系，投影前必须转回去。
void drawFourArmorReprojections(cv::Mat& frame, const kalfilter::KFResult& state, const Eigen::Matrix3d& R_C0C,
                                const cv::Mat& camera_matrix, const cv::Mat& distortion_coeffs, bool big = false);

}  // namespace draw

#endif
