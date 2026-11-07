# ifndef AUTOAIM_DRAW_
# define AUTOAIM_DRAW_

# include "armor_detector.hpp"
# include "kalman_filter_task.hpp"

#include <opencv2/core.hpp>



namespace draw {




void drawDetections(
    cv::Mat& frame,
    const detector::ArmorDetections& detections);

void drawStateText(
    cv::Mat& frame,
    const kalfilter::KFResult& state);

void drawFourArmorReprojections(
    cv::Mat& frame,
    const kalfilter::KFResult& state,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coeffs,
    bool big = false);



 







}//namespce draw












# endif