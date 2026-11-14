#ifndef AUTOAIM_DRAW_
#define AUTOAIM_DRAW_

#include <opencv2/core.hpp>

#include "estimation/kalman_filter.hpp"
#include "vision/armor_detector.hpp"

namespace draw {

void drawDetections(cv::Mat& frame, const detector::ArmorDetections& detections);

void drawStateText(cv::Mat& frame, const kalfilter::KFResult& state);

void drawFourArmorReprojections(cv::Mat& frame, const kalfilter::KFResult& state, const cv::Mat& camera_matrix,
                                const cv::Mat& distortion_coeffs, bool big = false);

}  // namespace draw

#endif
