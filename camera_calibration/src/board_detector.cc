# include "board_detector.hpp"

# include <algorithm>
# include <array>
# include <cmath>
# include <cstdio>
# include <optional>
# include <random>
# include <string>
# include <utility>
# include <vector>

# include <opencv2/calib3d.hpp>
# include <opencv2/imgproc.hpp>


namespace calib
{

namespace
{
// 第一轮用 FAST_CHECK 快速排掉明显没有棋盘的图；漏检了再关掉它重来一遍。
// 直接用不带 FAST_CHECK 的版本对大图会慢很多，这里两遍最省心。
constexpr int kFlagsFast = cv::CALIB_CB_ADAPTIVE_THRESH
                         | cv::CALIB_CB_NORMALIZE_IMAGE
                         | cv::CALIB_CB_FAST_CHECK;
constexpr int kFlagsFull = cv::CALIB_CB_ADAPTIVE_THRESH
                         | cv::CALIB_CB_NORMALIZE_IMAGE;

cv::Point toIntPoint(const cv::Point2f& p)
{
    return cv::Point(cvRound(p.x), cvRound(p.y));
}

}   // namespace


BoardDetector::BoardDetector(const BoardSpec& board, bool with_preview)
    : board_(board), with_preview_(with_preview)
{
}


bool BoardDetector::detect(const cv::Mat& image, const std::string& label, View& out)
{
    out = View {};
    if (image.empty()) return false;

    // findChessboardCorners 要单通道灰度图
    cv::Mat gray;
    if (image.channels() == 3)      { cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY); }
    else if (image.channels() == 4) { cv::cvtColor(image, gray, cv::COLOR_BGRA2GRAY); }
    else                            { gray = image; }
    if (gray.type() != CV_8UC1)
    {
        cv::Mat converted;
        gray.convertTo(converted, CV_8UC1);
        gray = converted;
    }

    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(gray, board_.patternSize(), corners, kFlagsFast);
    if (!found)
    {
        corners.clear();
        found = cv::findChessboardCorners(gray, board_.patternSize(), corners, kFlagsFull);
    }
    if (!found) return false;
    if (static_cast<int>(corners.size()) != board_.cols * board_.rows) return false;

    /*
     * 亚像素精修：findChessboardCorners 只给到整像素，误差 ±0.5 px 会直接抬高
     * 标定的重投影 RMS。cornerSubPix 在灰度梯度上找鞍点，通常能压到 0.1 px 级。
     * 窗口 11×11 对 1440×1080 的图比较合适；图很小可以调成 5×5。
     */
    cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));

    // 角点贴到图像边缘时，畸变模型在外推区不可靠，整张图直接弃用
    constexpr float kMargin = 8.0f;
    for (const auto& p : corners)
    {
        if (p.x < kMargin || p.y < kMargin ||
            p.x > static_cast<float>(gray.cols) - kMargin ||
            p.y > static_cast<float>(gray.rows) - kMargin) return false;
    }

    out.corners = corners;
    out.image_size = gray.size();
    out.label = label;

    if (with_preview_)
    {
        cv::Mat vis = image.clone();
        if (vis.channels() == 1) cv::cvtColor(vis, vis, cv::COLOR_GRAY2BGR);
        cv::drawChessboardCorners(vis, board_.patternSize(), corners, true);
        cv::putText(vis, label, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX,
                    0.8, cv::Scalar(0, 255, 255), 2);
        out.preview = vis;
    }
    return true;
}


double cornerShift(const View& a, const View& b)
{
    if (a.corners.empty() || a.corners.size() != b.corners.size()) return 0.0;

    double sum = 0.0;
    for (std::size_t i = 0; i < a.corners.size(); ++i)
    {
        sum += cv::norm(a.corners[i] - b.corners[i]);
    }
    return sum / static_cast<double>(a.corners.size());
}


std::vector<View> makeSyntheticViews(const BoardSpec& board,
                                     const cv::Mat& ref_K,
                                     const cv::Mat& ref_D,
                                     const cv::Size& image_size,
                                     int wanted,
                                     double noise_px)
{
    std::vector<View> views;
    if (ref_K.empty() || ref_D.empty() || image_size.width <= 0 || image_size.height <= 0)
        return views;

    // 把三维点平移到「以板中心为原点」，见头文件注释
    std::vector<cv::Point3f> object = board.objectPoints();
    const cv::Point3f centre(static_cast<float>((board.cols - 1) * board.square * 0.5),
                             static_cast<float>((board.rows - 1) * board.square * 0.5),
                             0.0f);
    for (auto& p : object) p -= centre;

    const double fx = ref_K.at<double>(0, 0);
    const double fy = ref_K.at<double>(1, 1);
    const double cx = ref_K.at<double>(0, 2);
    const double cy = ref_K.at<double>(1, 2);
    if (!(fx > 0.0 && fy > 0.0)) return views;

    // 让棋盘中心扫过画面 3×3 的九个区域 —— 标定时姿态覆盖越全，
    // 主点和畸变系数越不容易和焦距耦合在一起。
    const std::array<cv::Point2d, 9> centres
    {
        cv::Point2d(0.22, 0.22), cv::Point2d(0.50, 0.20), cv::Point2d(0.78, 0.22),
        cv::Point2d(0.20, 0.50), cv::Point2d(0.50, 0.50), cv::Point2d(0.80, 0.50),
        cv::Point2d(0.22, 0.78), cv::Point2d(0.50, 0.80), cv::Point2d(0.78, 0.78)
    };

    std::mt19937 rng(20260520u);                       // 固定种子，自检结果可复现
    std::uniform_real_distribution<double> uni(-1.0, 1.0);
    std::optional<std::normal_distribution<double>> gauss;
    if (noise_px > 0.0) gauss.emplace(0.0, noise_px);

    constexpr double kMargin = 25.0;
    int attempts = 0;
    const int max_attempts = wanted * 80;

    while (static_cast<int>(views.size()) < wanted && attempts < max_attempts)
    {
        ++attempts;

        const cv::Point2d target = centres[views.size() % centres.size()];
        const double tz = 900.0 + 1100.0 * (uni(rng) + 1.0) * 0.5;   // 0.9 ~ 2.0 m
        // 由「希望板中心出现在画面哪个像素」反推 tx、ty（针孔近似，够用了）
        const double tx = (target.x * image_size.width  - cx) * tz / fx + 60.0 * uni(rng);
        const double ty = (target.y * image_size.height - cy) * tz / fy + 60.0 * uni(rng);

        // 倾斜角 ±20° 左右，再加点面内旋转，保证不是一堆正对相机的平行板
        const double rx = 0.35 * uni(rng);
        const double ry = 0.35 * uni(rng);
        const double rz = 0.50 * uni(rng);

        const cv::Mat rvec = (cv::Mat_<double>(3, 1) << rx, ry, rz);
        const cv::Mat tvec = (cv::Mat_<double>(3, 1) << tx, ty, tz);

        std::vector<cv::Point2f> projected;
        cv::projectPoints(object, rvec, tvec, ref_K, ref_D, projected);
        if (projected.size() != object.size()) continue;

        bool inside = true;
        for (auto& p : projected)
        {
            if (gauss.has_value())
            {
                p.x += static_cast<float>((*gauss)(rng));
                p.y += static_cast<float>((*gauss)(rng));
            }
            if (p.x < kMargin || p.y < kMargin ||
                p.x > image_size.width  - kMargin ||
                p.y > image_size.height - kMargin) { inside = false; break; }
        }
        if (!inside) continue;

        View v;
        v.corners = projected;
        v.image_size = image_size;
        v.label = "synthetic_" + std::to_string(views.size());
        views.push_back(std::move(v));
    }

    return views;
}


cv::Mat drawSyntheticBoard(const View& view, const BoardSpec& board)
{
    cv::Mat image(view.image_size, CV_8UC3, cv::Scalar(238, 238, 238));
    const int cols = board.cols;
    const int rows = board.rows;
    const auto& p = view.corners;
    if (image.empty() || static_cast<int>(p.size()) != cols * rows) return image;

    // 用投影后的四个角点填出每个小格：这些格子天然带着畸变和透视，
    // 后面 cv::undistort 一跑就能直观看出来有没有把线掰直。
    for (int r = 0; r + 1 < rows; ++r)
    {
        for (int c = 0; c + 1 < cols; ++c)
        {
            if (((r + c) & 1) == 0) continue;
            const std::vector<cv::Point> quad
            {
                toIntPoint(p[r * cols + c]),
                toIntPoint(p[r * cols + c + 1]),
                toIntPoint(p[(r + 1) * cols + c + 1]),
                toIntPoint(p[(r + 1) * cols + c])
            };
            cv::fillConvexPoly(image, quad.data(), static_cast<int>(quad.size()),
                               cv::Scalar(28, 28, 28));
        }
    }

    const std::vector<cv::Point> border
    {
        toIntPoint(p[0]),
        toIntPoint(p[cols - 1]),
        toIntPoint(p[(rows - 1) * cols + cols - 1]),
        toIntPoint(p[(rows - 1) * cols])
    };
    for (int i = 0; i < 4; ++i)
    {
        cv::line(image, border[i], border[(i + 1) % 4], cv::Scalar(0, 0, 220), 2);
    }
    for (const auto& q : p)
    {
        cv::circle(image, toIntPoint(q), 2, cv::Scalar(0, 220, 0), -1);
    }
    return image;
}

}   // namespace calib
