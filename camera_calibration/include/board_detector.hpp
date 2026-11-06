# ifndef CAMERA_CALIBRATION_BOARD_DETECTOR_HPP
# define CAMERA_CALIBRATION_BOARD_DETECTOR_HPP

# include <string>
# include <vector>

# include <opencv2/core.hpp>

# include "calib_config.hpp"


namespace calib
{

/*
 * 一张「有效标定视图」= 一幅图像里成功检出的全部内角点。
 * corners 已经过亚像素精修，顺序与 BoardSpec::objectPoints() 一一对应。
 */
struct View
{
    std::vector<cv::Point2f> corners {};
    cv::Size image_size {};
    std::string label {};       // 文件名 / 帧号 / synthetic_XX，只用于打印和 CSV
    cv::Mat preview {};         // 画了角点的 BGR 图，按需填充，空表示没存
};


class BoardDetector
{
    public:
    explicit BoardDetector(const BoardSpec& board, bool with_preview = false);

    // 在单张图上找棋盘；成功返回 true 并填好 out
    bool detect(const cv::Mat& image, const std::string& label, View& out);

    const BoardSpec& board() const noexcept { return board_; }

    private:
    BoardSpec board_;
    bool with_preview_;
};


// 两张视图的平均角点位移（px）。视频/摄像头采集时用它跳过几乎重复的画面，
// 否则连拍 40 张同一个姿势，标定方程会病态（内参互相耦合、k1/k2 乱跑）。
double cornerShift(const View& a, const View& b);


/*
 * 零素材自检：以 ref_K / ref_D 为真值，把棋盘摆成各种位置 + 倾角，
 * 用 cv::projectPoints 投影出「假角点观测」。
 * 程序拿这些观测去标定，结果应当几乎精确等于真值 —— 这能一次性验证
 * 三维点顺序、投影模型、畸变系数顺序、标定 flags 有没有搞错。
 *
 * 注意：这里用的三维点以「板中心」为原点（比 objectPoints() 少一个平移），
 * 平移只影响外参 tvec，不影响内参，所以完全合法，但姿态更好控制。
 */
std::vector<View> makeSyntheticViews(const BoardSpec& board,
                                     const cv::Mat& ref_K,
                                     const cv::Mat& ref_D,
                                     const cv::Size& image_size,
                                     int wanted,
                                     double noise_px);


// 按投影后的角点把棋盘格画出来（自带透视 + 畸变），给去畸变预览当素材
cv::Mat drawSyntheticBoard(const View& view, const BoardSpec& board);

}   // namespace calib

# endif
