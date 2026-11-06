# ifndef CAMERA_CALIBRATION_CALIB_CONFIG_HPP
# define CAMERA_CALIBRATION_CALIB_CONFIG_HPP

# include <string>
# include <vector>

# include <opencv2/core.hpp>
# include <opencv2/core/types.hpp>


namespace calib
{

/*
 * 棋盘格规格。
 *
 * 注意 cols / rows 填的是「内角点」个数，不是格子数：
 *   一张 12 × 9 个小方格的棋盘 => 内角点是 11 × 8 => cols=11, rows=8。
 * 数错了 findChessboardCorners 会一直返回 false，这是新手最常踩的坑。
 *
 * square 的单位随便用（mm / m / 任意单位都行），因为它只影响外参 tvec 的单位，
 * 不会影响内参 K。但全工程必须统一，这里统一用 mm，和 fixed 版装甲板模板一致。
 */
struct BoardSpec
{
    int cols = 11;             // 横向内角点数
    int rows = 8;              // 纵向内角点数
    double square = 25.0;      // 单个方格边长（mm）

    cv::Size patternSize() const noexcept { return cv::Size(cols, rows); }

    // 生成标定用的三维点：板放在 z=0 平面，逐行从左到右、从上到下。
    // 顺序必须与 cv::findChessboardCorners 返回的二维角点顺序一一对应。
    std::vector<cv::Point3f> objectPoints() const;
};


// 输入源。Synthetic 不需要任何图片素材：
// 用 fixed 版给的相机内参当「真值」，把棋盘按各种姿态投影出一堆假角点，
// 再让程序自己标定回来 —— 用来零素材验证整条标定链路是否正确。
enum class Source { Images, Video, Camera, Synthetic };


struct Options
{
    Source source = Source::Images;
    std::string input = "images";                       // 图片目录 / 视频文件 / 摄像头序号
    std::string output = "output";
    std::string reference = "config/reference_intrinsics.yml";

    BoardSpec board {};

    // ---- 标定模型开关（直接翻译成 cv::calibrateCamera 的 flags）----
    bool fix_k3 = false;                // 固定 k3 = 0（fixed 版给的畸变里 k3 就是 0）
    bool fix_principal_point = false;   // 主点固定在图像中心，数据少时可以防止乱跑
    bool zero_tangent_dist = false;     // 强制 p1 = p2 = 0

    // ---- 输出开关 ----
    bool preview = false;               // 保存画了角点的检测图
    bool compare = true;                // 与 fixed 版内参对照
    bool refine_outliers = true;        // 剔除重投影误差离群的视图后重标一次
    bool save_undistort = true;         // 保存去畸变前后对比图

    // ---- 采集 / 筛选 ----
    int min_views = 8;                  // 少于这个数就拒绝出结果
    int max_views = 40;                 // 最多用多少张（太多反而慢且容易堆重复姿态）
    double min_move_px = 25.0;          // 视频/摄像头：与上一张角点平均位移小于它就跳过
    int frame_stride = 3;               // 视频每隔几帧检测一次
    int camera_id = 0;
    int synthetic_views = 20;
    double synthetic_noise = 0.0;       // 合成角点的高斯噪声 σ（px），0 = 完全理想观测

    bool help = false;
    bool ok = true;                     // 参数解析是否成功
    std::string error {};               // 解析失败原因
};


Options parseOptions(int argc, char** argv);
void printUsage(const char* argv0);

}   // namespace calib

# endif
