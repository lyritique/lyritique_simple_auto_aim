# include "draw.hpp"

# include <array>
# include <cmath>
# include <iomanip>
# include <sstream>
# include <string>
# include <vector>

# include <opencv2/calib3d.hpp>
# include <opencv2/imgproc.hpp>









namespace draw {

namespace  {
cv::Scalar getcolor(int color)
{
    switch (color)
    {
    case 0: return {0, 165, 255};      //蓝对应橙
    case 1: return {255, 255, 255};    //红对应白
    case 2: return {0, 0, 0};          //白对应黑
    case 3: return {0, 255, 0};        //紫对应绿
    default: return {0, 255, 0};
    }
}



double wrap(double angle)//我怎么记得我写过呢
{
    constexpr double kPi = 3.14159265358979323846;

    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;

    return angle;
}


}



//detector
void drawDetections(
    cv::Mat& frame,
    const detector::ArmorDetections& detections)
{
    //每个装甲板依次
    for (const auto& detection : detections)
    {
        //构造舍入后的整数点
        std::vector<cv::Point> points;
        points.reserve(4);

        for (const cv::Point2f& corner : detection.corners)
        {
            points.emplace_back(
                cvRound(corner.x),
                cvRound(corner.y));
        }

        //画外轮廓
        cv::polylines(
            frame, points, true, getcolor(detection.color), 2);

        //第一个点，左上
        const cv::Point origin = points.front();

        std::ostringstream label;
        label << detection.number << " "
              << std::fixed << std::setprecision(2)
              << detection.score;

        cv::putText(
            frame,
            label.str(),
            {origin.x, std::max(15, origin.y - 5)},
            cv::FONT_HERSHEY_SIMPLEX,
            0.5,
            getcolor(detection.color),
            1,
            cv::LINE_AA);//抗锯齿
    }
}


//右上角ekf信息
void drawStateText(
    cv::Mat& frame,
    const kalfilter::KFResult& state)
{
    //准备5行文字
    std::array<std::string, 5> lines;

    if (!state.valid)
    {
        lines = {
            "EKF: not initialized",
            "",
            "",
            "",
            ""
        };
    }
    else//输入九维状态
    {
        std::ostringstream                             position;
        position << std::fixed << std::setprecision(2)
                 << "pos: "
                 << state.state(0) << ", "
                 << state.state(1) << ", "
                 << state.state(2);

        std::ostringstream                            velocity;
        velocity << std::fixed << std::setprecision(2)
                 << "vel: "
                 << state.state(4) << ", "
                 << state.state(5) << ", "
                 << state.state(6);

        std::ostringstream                             yaw;
        yaw << std::fixed << std::setprecision(2)
            << "yaw: " << state.state(3);

        std::ostringstream yaw_rate;
        yaw_rate << std::fixed << std::setprecision(2)
                 << "vyaw: " << state.state(7);

        std::ostringstream                             radius;
        radius << std::fixed << std::setprecision(3)
               << "r: " << state.state(8);

        lines = {
            position.str(),
            velocity.str(),
            yaw.str(),
            yaw_rate.str(),
            radius.str()
        };
    }

    constexpr int line_height {22};
    constexpr int margin  {8};

    //计算每行长度，取最长
    int text_width = 0;
    for (const auto& line : lines)
    {
        int baseline = 0;
        const cv::Size size = cv::getTextSize(
            line,
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            1,
            &baseline);

        text_width = std::max(text_width, size.width);
    }

    const int panel_width = text_width + 2 * margin;
    const int panel_height =
        static_cast<int>(lines.size()) * line_height + margin;

    //构造矩形背景（黑色）
    const cv::Rect back(
        std::max(0, frame.cols - panel_width),
        0,
        std::min(panel_width, frame.cols),
        std::min(panel_height, frame.rows));

    cv::rectangle(frame, back, cv::Scalar(0, 0, 0), cv::FILLED);

    //写字
    for (std::size_t i = 0; i < lines.size(); ++i)
    {
        cv::putText(
            frame,
            lines[i],
            {back.x + margin,
             back.y + margin + static_cast<int>(i) * line_height},
            cv::FONT_HERSHEY_SIMPLEX,
            0.55,
            cv::Scalar(0, 255, 0),
            1,
            cv::LINE_AA);
    }
}

void drawFourArmorReprojections(
    cv::Mat& frame,
    const kalfilter::KFResult& state,
    const cv::Mat& camera_matrix,
    const cv::Mat& distortion_coeffs,
    bool big)
{
    if (!state.valid) return;

    constexpr double kPi = 3.14159265358979323846;

    const double px = state.state(0);
    const double py = state.state(1);
    const double pz = state.state(2);
    const double chassis_yaw = state.state(3);
    const double radius = state.state(8);

    const double half_width = big ? 225.0 * 0.5 / 1000.0
                                  : 135.0 * 0.5 / 1000.0;
    const double half_height = 55.0 * 0.5 / 1000.0;

    // 与当前 PnP 对象点保持一致：
    // P0(0,-w,+h), P1(0,-w,-h),
    // P2(0,+w,-h), P3(0,+w,+h)
    const std::vector<cv::Point3d> object_points {
        {0.0, -half_width,  half_height},
        {0.0, -half_width, -half_height},
        {0.0,  half_width, -half_height},
        {0.0,  half_width,  half_height},
    };

    const std::array<cv::Scalar, 4> colors {
        cv::Scalar(0, 255, 0),
        cv::Scalar(0, 255, 255),
        cv::Scalar(255, 0, 0),
        cv::Scalar(255, 0, 255),
    };

    for (int armor_id = 0; armor_id < 4; ++armor_id)
    {
        //const double theta = wrap(
        //    chassis_yaw + armor_id * kPi * 0.5);

        const double theta = chassis_yaw + armor_id * kPi * 0.5;

        const double center_x = px + radius * std::sin(theta);

        const double center_z = pz + radius * std::cos(theta);

        // 与 EKF 观测模型的板中心公式一致
        //const double center_x =
        //    px + radius * std::sin(theta);
        const double center_y = py;
        //const double center_z =
        //    pz + radius * std::cos(theta);

        const double s = std::sin(theta);
        const double c = std::cos(theta);

        // 装甲板坐标系 -> 相机坐标系
        // 第 0 列：板法向
        // 第 1 列：板宽度方向
        // 第 2 列：板高度方向，当前约定为相机 y 向下
        const cv::Mat rotation =
            (cv::Mat_<double>(3, 3) <<
                s,  c, 0.0,
                0.0, 0.0, 1.0,
                c, -s, 0.0);

        cv::Mat rvec;
        cv::Rodrigues(rotation, rvec);

        const cv::Mat tvec =
            (cv::Mat_<double>(3, 1) <<
                center_x,
                center_y,
                center_z);

        std::vector<cv::Point2d> projected;
        cv::projectPoints(
            object_points,
            rvec,
            tvec,
            camera_matrix,
            distortion_coeffs,
            projected);

        if (projected.size() != 4) continue;

        std::vector<cv::Point> image_points;
        image_points.reserve(4);

        for (const auto& point : projected)
        {
            image_points.emplace_back(
                cvRound(point.x),
                cvRound(point.y));
        }

        cv::polylines(
            frame,
            image_points,
            true,
            colors[armor_id],
            2,
            cv::LINE_AA);

        cv::circle(
            frame,
            image_points[0],
            4,
            colors[armor_id],
            cv::FILLED);

        cv::putText(
            frame,
            "id=" + std::to_string(armor_id),
            image_points[0] + cv::Point(5, -5),
            cv::FONT_HERSHEY_SIMPLEX,
            0.45,
            colors[armor_id],
            1,
            cv::LINE_AA);

        //看完画的，心脏也不跳了
    }
}







}
