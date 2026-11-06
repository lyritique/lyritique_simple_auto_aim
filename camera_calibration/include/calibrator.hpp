# ifndef CAMERA_CALIBRATION_CALIBRATOR_HPP
# define CAMERA_CALIBRATION_CALIBRATOR_HPP

# include <string>
# include <vector>

# include <opencv2/core.hpp>

# include "board_detector.hpp"
# include "calib_config.hpp"


namespace calib
{

/*
 * 张正友平面标定（cv::calibrateCamera）的结果 + 一堆便于判断好坏的统计量。
 * 所有数组都按「输入 views 的原始下标」排列，被剔除的视图 per_view_rms 记 -1。
 */
struct CalibrationResult
{
    cv::Mat K {};                       // 3×3 内参矩阵，CV_64F
    cv::Mat D {};                       // 1×5 畸变 (k1,k2,p1,p2,k3)，CV_64F
    cv::Size image_size {};
    double rms = 0.0;                   // OpenCV 给的整体重投影 RMS（px）

    std::vector<double> per_view_rms {};      // 最终模型下每张图的误差
    std::vector<double> first_pass_rms {};    // 第一轮（未剔除）模型下每张图的误差
    std::vector<double> tilt_deg {};          // 每张图里板面相对相机的倾角（度）
    std::vector<bool>   used {};              // 是否参与最终标定
    std::vector<cv::Mat> rvecs {}, tvecs {}; // 每个保留视图的外参

    int used_views = 0;
    int passes = 0;                     // 实际标定了几轮（1 = 没剔除，2 = 剔除后重标）
    bool ok = false;
    std::string message {};
};


class Calibrator
{
    public:
    /*
     * flags   : 原样传给 cv::calibrateCamera
     * reject_sigma : 用「中位数 + sigma × 1.4826 × MAD」判离群，<=0 表示不剔除
     * refine_outliers : 是否真的执行「剔掉坏图再标一次」
     */
    static CalibrationResult run(const std::vector<View>& views,
                                 const BoardSpec& board,
                                 int flags,
                                 double reject_sigma,
                                 bool refine_outliers);
};

}   // namespace calib

# endif
