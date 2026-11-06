# ifndef CAMERA_CALIBRATION_CALIB_IO_HPP
# define CAMERA_CALIBRATION_CALIB_IO_HPP

# include <string>
# include <vector>

# include <opencv2/core.hpp>

# include "board_detector.hpp"
# include "calib_config.hpp"
# include "calibrator.hpp"


namespace calib
{

// 一套可以独立读写的相机内参（用来跟 fixed 版给的参考值做对照）
struct Intrinsics
{
    cv::Mat K {}, D {};
    cv::Size size {};
    double rms = 0.0;
    std::string name {};
    bool valid = false;
};

// 从 OpenCV YAML/XML 读内参；字段与 output/calibration.yml 完全同名，可互换
bool loadIntrinsics(const std::string& path, Intrinsics& out);

// 读不到参考文件时的兜底：直接抄 kalman_filter_fixed 的 pnpConfig::camParams
Intrinsics fixedFallback();

// ---- 落盘 ----
bool saveCalibration(const std::string& path, const CalibrationResult& result,
                     const Options& opt, const std::vector<View>& views);
bool saveSnippet(const std::string& path, const CalibrationResult& result);
bool savePerViewCsv(const std::string& path, const CalibrationResult& result,
                    const std::vector<View>& views);
bool saveUndistortPreview(const std::string& path, const cv::Mat& image,
                          const CalibrationResult& result);
int  savePreviews(const std::string& dir, const std::vector<View>& views, int max_save);

// ---- 终端报告 ----
void printSummary(const CalibrationResult& result, const std::vector<View>& views);
void printComparison(const Intrinsics& reference, const CalibrationResult& result);

}   // namespace calib

# endif
