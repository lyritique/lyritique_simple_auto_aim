# include "calib_io.hpp"

# include <algorithm>
# include <array>
# include <cmath>
# include <cstdio>
# include <ctime>
# include <fstream>
# include <iomanip>
# include <sstream>
# include <string>
# include <vector>

# include <opencv2/calib3d.hpp>
# include <opencv2/core/persistence.hpp>
# include <opencv2/imgcodecs.hpp>
# include <opencv2/imgproc.hpp>


namespace calib
{

namespace
{

std::string nowString()
{
    const std::time_t t = std::time(nullptr);
    char buf[64] {};
    const std::tm* local = std::localtime(&t);
    if (local != nullptr) std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", local);
    else std::snprintf(buf, sizeof(buf), "unknown");
    return std::string(buf);
}


// 畸变矩阵可能是 1×5、5×1、甚至别的形状，统一摊平成 1×N 的 double 再取值
double distAt(const cv::Mat& D, int k)
{
    if (D.empty()) return 0.0;
    cv::Mat tmp;
    if (D.type() != CV_64FC1) D.convertTo(tmp, CV_64FC1);
    else tmp = D.clone();
    const cv::Mat flat = tmp.reshape(1, 1);
    return static_cast<int>(flat.total()) > k ? flat.at<double>(0, k) : 0.0;
}


std::string sourceName(Source s)
{
    switch (s)
    {
        case Source::Images:    return "images";
        case Source::Video:     return "video";
        case Source::Camera:    return "camera";
        case Source::Synthetic: return "synthetic";
    }
    return "unknown";
}


std::string flagsName(const Options& opt)
{
    std::string s = "default";
    if (opt.fix_k3)               s += " + CALIB_FIX_K3";
    if (opt.fix_principal_point)  s += " + CALIB_FIX_PRINCIPAL_POINT";
    if (opt.zero_tangent_dist)    s += " + CALIB_ZERO_TANGENT_DIST";
    return s;
}


std::string padIndex(int i)
{
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d", i);
    return std::string(buf);
}


// 把两张图横向拼起来；cv::putText 画不了中文，标注用英文避免乱码
bool writePair(const std::string& path, const cv::Mat& a, const cv::Mat& b,
               const std::string& left_text, const std::string& right_text)
{
    cv::Mat left = a.clone();
    cv::Mat right = b.clone();
    constexpr int kMaxSingleWidth = 800;
    if (left.cols > kMaxSingleWidth)
    {
        const double scale = static_cast<double>(kMaxSingleWidth) / left.cols;
        cv::resize(left, left, cv::Size(), scale, scale);
        cv::resize(right, right, cv::Size(), scale, scale);
    }

    cv::Mat canvas;
    cv::hconcat(left, right, canvas);
    cv::putText(canvas, left_text, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX,
                0.9, cv::Scalar(0, 0, 235), 2);
    cv::putText(canvas, right_text, cv::Point(left.cols + 12, 30),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 170, 0), 2);
    return cv::imwrite(path, canvas);
}

}   // namespace


bool loadIntrinsics(const std::string& path, Intrinsics& out)
{
    try
    {
        cv::FileStorage fs(path, cv::FileStorage::READ);
        if (!fs.isOpened()) return false;

        cv::Mat K, D;
        fs["camera_matrix"] >> K;
        fs["distortion_coefficients"] >> D;
        if (K.empty() || D.empty() || K.total() != 9) return false;

        if (K.type() != CV_64FC1) K.convertTo(K, CV_64FC1);
        if (D.type() != CV_64FC1) D.convertTo(D, CV_64FC1);
        out.K = K.clone();
        out.D = D.clone().reshape(1, 1);      // 统一成 1×N

        int w = 0, h = 0;
        cv::FileNode node_w = fs["image_width"];
        cv::FileNode node_h = fs["image_height"];
        if (!node_w.empty()) node_w >> w;
        if (!node_h.empty()) node_h >> h;
        out.size = cv::Size(w, h);

        cv::FileNode node_rms = fs["reference_rms"];
        if (!node_rms.empty()) node_rms >> out.rms;
        else
        {
            node_rms = fs["rms"];
            if (!node_rms.empty()) node_rms >> out.rms;
        }

        out.name = path;
        out.valid = true;
        return true;
    }
    catch (const cv::Exception&)
    {
        return false;
    }
}


Intrinsics fixedFallback()
{
    Intrinsics ref;
    ref.K = (cv::Mat_<double>(3, 3) <<
        2362.2795952460974, 0.0, 679.17513921636282,
        0.0, 2336.1916035376717, 529.06858905837635,
        0.0, 0.0, 1.0);
    ref.D = (cv::Mat_<double>(1, 5) <<
        -0.098561592108101559, 0.11173066531932681,
        -0.0038352945867108721, -0.0020421443354436304, 0.0);
    ref.size = cv::Size(1440, 1080);
    ref.rms = 0.1448;
    ref.name = "kalman_filter_fixed / material/说明.txt（内置兜底值）";
    ref.valid = true;
    return ref;
}


bool saveCalibration(const std::string& path, const CalibrationResult& result,
                     const Options& opt, const std::vector<View>& views)
{
    if (!result.ok) return false;
    try
    {
        cv::FileStorage fs(path, cv::FileStorage::WRITE);
        if (!fs.isOpened()) return false;

        // 字段名和 config/reference_intrinsics.yml 保持一致，
        // 这份文件可以直接被别的 OpenCV 程序（包括改造后的 fixed 版）读走。
        fs << "calibration_time" << nowString();
        fs << "source" << sourceName(opt.source);
        fs << "flags" << flagsName(opt);
        fs << "image_width" << result.image_size.width;
        fs << "image_height" << result.image_size.height;
        fs << "board_cols" << opt.board.cols;
        fs << "board_rows" << opt.board.rows;
        fs << "square_mm" << opt.board.square;
        fs << "total_views" << static_cast<int>(views.size());
        fs << "used_views" << result.used_views;
        fs << "passes" << result.passes;
        fs << "rms" << result.rms;
        fs << "camera_matrix" << result.K;
        fs << "distortion_coefficients" << result.D;

        fs << "labels" << "[";
        for (const auto& v : views) fs << v.label;
        fs << "]";
        fs << "per_view_rms" << "[";
        for (const double e : result.per_view_rms) fs << e;
        fs << "]";
        fs.release();
        return true;
    }
    catch (const cv::Exception&)
    {
        return false;
    }
}


bool saveSnippet(const std::string& path, const CalibrationResult& result)
{
    if (!result.ok) return false;
    std::ofstream file(path);
    if (!file) return false;

    std::ostringstream os;
    os << std::setprecision(16);

    os << "// ============================================================\n"
       << "// 由 camera_calibration 自动生成，用于回填 kalman_filter_fixed\n"
       << "// 标定 RMS = " << result.rms << " px, 图像 = "
       << result.image_size.width << "x" << result.image_size.height
       << ", 有效视图 = " << result.used_views << " 张\n"
       << "// ============================================================\n\n"
       << "// ---------- 方案 A：只改数值（最省事）----------\n"
       << "// 用下面这段替换 include/data_interaction.hpp 里的 pnpConfig::camParams\n"
       << "struct camParams\n"
       << "{\n"
       << "    double width  = " << result.image_size.width << ";\n"
       << "    double height = " << result.image_size.height << ";\n"
       << "    double fx     = " << result.K.at<double>(0, 0) << ";\n"
       << "    double fy     = " << result.K.at<double>(1, 1) << ";\n"
       << "    double cx     = " << result.K.at<double>(0, 2) << ";\n"
       << "    double cy     = " << result.K.at<double>(1, 2) << ";\n"
       << "    Eigen::Matrix<double, 5, 1> dist  {"
       << distAt(result.D, 0) << ", " << distAt(result.D, 1) << ", "
       << distAt(result.D, 2) << ", " << distAt(result.D, 3) << ", "
       << distAt(result.D, 4) << "};\n"
       << "};\n\n"
       << "// ---------- 方案 B：让 fixed 版直接读 YAML（推荐长期使用）----------\n"
       << "// 先把 output/calibration.yml 拷到 fixed 版工程根目录，\n"
       << "// 再把 src/data_interaction.cc 里 pnpSolver::pnpSolver() 的函数体换成下面这段，\n"
       << "// 并在文件顶部加 # include <opencv2/core/persistence.hpp>\n"
       << "//\n"
       << "// pnpSolver::pnpSolver()\n"
       << "// {\n"
       << "//     cv::FileStorage fs(\"calibration.yml\", cv::FileStorage::READ);\n"
       << "//     if (!fs.isOpened()) throw std::runtime_error(\"读不到 calibration.yml\");\n"
       << "//     fs[\"camera_matrix\"] >> camera_matrix_;\n"
       << "//     fs[\"distortion_coefficients\"] >> distortion_coeffs_;\n"
       << "//     if (camera_matrix_.empty() || distortion_coeffs_.empty())\n"
       << "//         throw std::runtime_error(\"内参文件内容不完整\");\n"
       << "// }\n"
       << "//\n"
       << "// 注意：畸变向量顺序是 OpenCV 标准的 (k1, k2, p1, p2, k3)，\n"
       << "// 与 pnpConfig::camParams::dist 的顺序完全一致，不需要重排。\n";

    file << os.str();
    return static_cast<bool>(file);
}


bool savePerViewCsv(const std::string& path, const CalibrationResult& result,
                    const std::vector<View>& views)
{
    std::ofstream csv(path);
    if (!csv) return false;

    csv << "view,label,used,rms_first_pass,rms_final,tilt_deg,centre_x_px,centre_y_px\n";
    csv << std::fixed << std::setprecision(5);

    for (std::size_t i = 0; i < views.size(); ++i)
    {
        double mx = 0.0, my = 0.0;
        for (const auto& p : views[i].corners) { mx += p.x; my += p.y; }
        if (!views[i].corners.empty())
        {
            mx /= static_cast<double>(views[i].corners.size());
            my /= static_cast<double>(views[i].corners.size());
        }

        csv << i << ',' << views[i].label << ','
            << (i < result.used.size() && result.used[i] ? 1 : 0) << ','
            << (i < result.first_pass_rms.size() ? result.first_pass_rms[i] : -1.0) << ','
            << (i < result.per_view_rms.size() ? result.per_view_rms[i] : -1.0) << ','
            << (i < result.tilt_deg.size() ? result.tilt_deg[i] : 0.0) << ','
            << mx << ',' << my << '\n';
    }
    return static_cast<bool>(csv);
}


bool saveUndistortPreview(const std::string& path, const cv::Mat& image,
                          const CalibrationResult& result)
{
    if (image.empty() || !result.ok) return false;
    cv::Mat undistorted;
    cv::undistort(image, undistorted, result.K, result.D);
    return writePair(path, image, undistorted, "with distortion", "undistorted");
}


int savePreviews(const std::string& dir, const std::vector<View>& views, int max_save)
{
    int saved = 0;
    for (const auto& v : views)
    {
        if (saved >= max_save) break;
        if (v.preview.empty()) continue;
        const std::string path = dir + "/corners_" + padIndex(saved) + ".jpg";
        if (cv::imwrite(path, v.preview)) ++saved;
    }
    return saved;
}


void printSummary(const CalibrationResult& result, const std::vector<View>& views)
{
    std::printf("\n========== 标定结果 ==========\n");
    std::printf("输入视图        : %zu 张\n", views.size());
    std::printf("实际参与标定    : %d 张（剔除 %zu 张，共 %d 轮）\n",
                result.used_views, views.size() - static_cast<std::size_t>(result.used_views),
                result.passes);
    std::printf("图像尺寸        : %d x %d\n", result.image_size.width, result.image_size.height);
    std::printf("整体重投影 RMS  : %.4f px   (<0.5 优秀 / <1.0 可用 / >2.0 建议重拍)\n",
                result.rms);

    double mean = 0.0, worst = 0.0;
    int count = 0;
    for (const double e : result.per_view_rms)
    {
        if (e < 0.0) continue;
        mean += e;
        worst = std::max(worst, e);
        ++count;
    }
    if (count > 0) mean /= count;
    std::printf("逐视图 RMS      : 平均 %.4f px / 最差 %.4f px\n", mean, worst);

    double tilt_min = 1e9, tilt_max = 0.0;
    std::array<bool, 9> cells {};
    int cell_count = 0;
    for (std::size_t i = 0; i < views.size(); ++i)
    {
        if (i >= result.used.size() || !result.used[i]) continue;
        tilt_min = std::min(tilt_min, result.tilt_deg[i]);
        tilt_max = std::max(tilt_max, result.tilt_deg[i]);

        double cx = 0.0, cy = 0.0;
        for (const auto& p : views[i].corners) { cx += p.x; cy += p.y; }
        if (views[i].corners.empty()) continue;
        cx /= static_cast<double>(views[i].corners.size()) * result.image_size.width;
        cy /= static_cast<double>(views[i].corners.size()) * result.image_size.height;
        const int gx = std::clamp(static_cast<int>(cx * 3.0), 0, 2);
        const int gy = std::clamp(static_cast<int>(cy * 3.0), 0, 2);
        const int id = gy * 3 + gx;
        if (!cells[static_cast<std::size_t>(id)]) { cells[static_cast<std::size_t>(id)] = true; ++cell_count; }
    }
    std::printf("板面倾角范围    : %.1f ~ %.1f deg（全是接近 0 度的正面板 => 内参不可信）\n",
                count > 0 ? tilt_min : 0.0, tilt_max);
    std::printf("棋盘中心覆盖    : %d/9 个画面分区（建议 >= 6，四角都要拍到）\n", cell_count);

    std::printf("\ncamera_matrix (K):\n");
    std::printf("  [ %.10f, %10.6f, %.10f ]\n",
                result.K.at<double>(0, 0), result.K.at<double>(0, 1), result.K.at<double>(0, 2));
    std::printf("  [ %10.6f, %.10f, %.10f ]\n",
                result.K.at<double>(1, 0), result.K.at<double>(1, 1), result.K.at<double>(1, 2));
    std::printf("  [ %10.6f, %10.6f, %10.6f ]\n",
                result.K.at<double>(2, 0), result.K.at<double>(2, 1), result.K.at<double>(2, 2));
    std::printf("distortion (k1, k2, p1, p2, k3):\n");
    std::printf("  [ %.10f, %.10f, %.10f, %.10f, %.10f ]\n",
                distAt(result.D, 0), distAt(result.D, 1), distAt(result.D, 2),
                distAt(result.D, 3), distAt(result.D, 4));
    std::printf("\n%s\n", result.message.c_str());
}


void printComparison(const Intrinsics& ref, const CalibrationResult& got)
{
    if (!ref.valid || !got.ok || ref.K.total() != 9) return;

    std::printf("\n========== 与 fixed 版给定内参对照 ==========\n");
    std::printf("参考来源: %s（其标定 RMS %.4f px）\n", ref.name.c_str(), ref.rms);
    std::printf("  %-6s %16s %16s %14s\n", "参数", "fixed 参考值", "本次标定值", "差值");

    auto row = [](const char* name, double a, double b)
    {
        std::printf("  %-6s %16.6f %16.6f %14.6f\n", name, a, b, b - a);
    };

    row("fx", ref.K.at<double>(0, 0), got.K.at<double>(0, 0));
    row("fy", ref.K.at<double>(1, 1), got.K.at<double>(1, 1));
    row("cx", ref.K.at<double>(0, 2), got.K.at<double>(0, 2));
    row("cy", ref.K.at<double>(1, 2), got.K.at<double>(1, 2));
    const char* names[5] = {"k1", "k2", "p1", "p2", "k3"};
    for (int k = 0; k < 5; ++k) row(names[k], distAt(ref.D, k), distAt(got.D, k));

    /*
     * 像素级对照：把画面上 11×11 个像素点，先用参考内参反投影成无畸变射线，
     * 再用本次标定的内参（含畸变）投回像素，看同一个空间方向会落到哪里。
     * 这比单纯比 fx 的数值更能反映「换了内参，PnP 会差多少像素」。
     */
    const cv::Size size = (ref.size.width > 0 && ref.size.height > 0)
                        ? ref.size : got.image_size;
    constexpr int kGrid = 11;
    std::vector<cv::Point2f> grid;
    grid.reserve(kGrid * kGrid);
    for (int y = 0; y < kGrid; ++y)
    {
        for (int x = 0; x < kGrid; ++x)
        {
            grid.emplace_back(
                static_cast<float>(size.width  * (0.05 + 0.90 * x / (kGrid - 1))),
                static_cast<float>(size.height * (0.05 + 0.90 * y / (kGrid - 1))));
        }
    }

    double rms_px = -1.0, max_px = -1.0;
    try
    {
        std::vector<cv::Point2f> normalized;
        // 不传 P => 输出「去掉畸变后的归一化坐标」，正好是相机射线方向
        cv::undistortPoints(grid, normalized, ref.K, ref.D);

        std::vector<cv::Point3f> rays;
        rays.reserve(normalized.size());
        for (const auto& p : normalized) rays.emplace_back(p.x, p.y, 1.0f);

        std::vector<cv::Point2f> projected;
        const cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);
        cv::projectPoints(rays, zero, zero, got.K, got.D, projected);

        if (projected.size() == grid.size())
        {
            double sq = 0.0;
            max_px = 0.0;
            for (std::size_t i = 0; i < grid.size(); ++i)
            {
                const cv::Point2f d = projected[i] - grid[i];
                const double d2 = static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y;
                sq += d2;
                max_px = std::max(max_px, std::sqrt(d2));
            }
            rms_px = std::sqrt(sq / static_cast<double>(grid.size()));
        }
    }
    catch (const cv::Exception&)
    {
        // 对照只是附加功能，失败不影响主流程
    }

    if (rms_px >= 0.0)
    {
        std::printf("\n换用本次内参后，同一空间射线在画面上的投影位移: RMS %.3f px / 最大 %.3f px\n",
                    rms_px, max_px);
        if (rms_px < 0.5)
            std::printf("判读: 与 fixed 版内参基本一致。\n");
        else
            std::printf("判读: 差异明显。若是 --synthetic 自检出现这种情况，说明标定链路有 bug；\n"
                        "      若来自真实照片，通常是分辨率/镜头不同，或姿态覆盖不够导致内参跑偏。\n");
    }
}

}   // namespace calib
