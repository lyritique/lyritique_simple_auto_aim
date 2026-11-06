# include "calib_config.hpp"
# include "board_detector.hpp"
# include "calibrator.hpp"
# include "calib_io.hpp"

# include <algorithm>
# include <cctype>
# include <cstdio>
# include <cstdlib>
# include <filesystem>
# include <stdexcept>
# include <string>
# include <system_error>
# include <utility>
# include <vector>

# include <opencv2/calib3d.hpp>
# include <opencv2/core.hpp>
# include <opencv2/highgui.hpp>
# include <opencv2/imgcodecs.hpp>
# include <opencv2/imgproc.hpp>
# include <opencv2/videoio.hpp>


namespace fs = std::filesystem;

namespace
{

std::string lowerCopy(std::string s)
{
    for (auto& ch : s) ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    return s;
}


bool isImageFile(const fs::path& p)
{
    const std::string ext = lowerCopy(p.extension().string());
    return ext == ".jpg"  || ext == ".jpeg" || ext == ".png" ||
           ext == ".bmp"  || ext == ".tif"  || ext == ".tiff";
}


// ---------- 输入源 1：图片目录 ----------
bool collectImages(const calib::Options& opt, calib::BoardDetector& detector,
                   std::vector<calib::View>& views, cv::Mat& preview_image)
{
    std::error_code ec;
    if (!fs::is_directory(opt.input, ec))
    {
        std::fprintf(stderr,
            "找不到图片目录: %s\n"
            "把棋盘格照片放进该目录，或改用 --video / --camera / --synthetic。\n",
            opt.input.c_str());
        return false;
    }

    std::vector<fs::path> files;
    for (const auto& entry : fs::directory_iterator(opt.input, ec))
    {
        if (entry.is_regular_file() && isImageFile(entry.path())) files.push_back(entry.path());
    }
    std::sort(files.begin(), files.end());
    if (files.empty())
    {
        std::fprintf(stderr, "目录 %s 里没有 jpg/png/bmp/tif 图片\n", opt.input.c_str());
        return false;
    }

    std::printf("图片目录: %s（共 %zu 个候选文件）\n", opt.input.c_str(), files.size());
    for (const auto& file : files)
    {
        if (static_cast<int>(views.size()) >= opt.max_views) break;

        const cv::Mat image = cv::imread(file.string(), cv::IMREAD_COLOR);
        if (image.empty())
        {
            std::printf("  [跳过] %s（读不出来）\n", file.filename().string().c_str());
            continue;
        }

        calib::View view;
        const std::string label = file.filename().string();
        if (!detector.detect(image, label, view))
        {
            std::printf("  [跳过] %s（没找到完整棋盘 / 角点贴边）\n", label.c_str());
            continue;
        }
        std::printf("  [收录] %s  %d 个角点\n", label.c_str(),
                    static_cast<int>(view.corners.size()));
        if (views.empty()) preview_image = image.clone();
        views.push_back(std::move(view));
    }
    return true;
}


// ---------- 输入源 2：视频抽帧 ----------
bool collectVideo(const calib::Options& opt, calib::BoardDetector& detector,
                  std::vector<calib::View>& views, cv::Mat& preview_image)
{
    cv::VideoCapture cap(opt.input);
    if (!cap.isOpened())
    {
        std::fprintf(stderr, "打不开视频: %s\n", opt.input.c_str());
        return false;
    }

    std::printf("视频: %s（每 %d 帧检测一次，位移需 > %.1f px）\n",
                opt.input.c_str(), opt.frame_stride, opt.min_move_px);
    cv::Mat frame;
    int frame_index = 0;
    while (static_cast<int>(views.size()) < opt.max_views)
    {
        if (!cap.read(frame) || frame.empty()) break;
        const int current = frame_index++;
        if (current % opt.frame_stride != 0) continue;

        calib::View view;
        if (!detector.detect(frame, "frame_" + std::to_string(current), view)) continue;
        // 同一姿势连拍没用，还会让标定方程病态
        if (!views.empty() && calib::cornerShift(views.back(), view) < opt.min_move_px) continue;

        std::printf("  [收录] frame %d（第 %zu 张）\n", current, views.size() + 1);
        if (views.empty()) preview_image = frame.clone();
        views.push_back(std::move(view));
    }
    cap.release();
    return true;
}


// ---------- 输入源 3：摄像头实时采集 ----------
bool collectCamera(const calib::Options& opt, calib::BoardDetector& detector,
                   std::vector<calib::View>& views, cv::Mat& preview_image)
{
    if (std::getenv("DISPLAY") == nullptr && std::getenv("WAYLAND_DISPLAY") == nullptr)
    {
        std::fprintf(stderr,
            "[提示] 当前环境没检测到图形界面，实时窗口可能打不开。\n"
            "       纯命令行/WSL 无 X 转发时请改用 --images / --video / --synthetic。\n");
    }

    cv::VideoCapture cap(opt.camera_id);
    if (!cap.isOpened())
    {
        std::fprintf(stderr, "打不开摄像头 %d\n", opt.camera_id);
        return false;
    }

    std::printf("摄像头已打开。移动/倾斜标定板，空格抓一帧，q 或 ESC 结束。\n"
                "建议至少 12 张：四角各几张 + 中间几张 + 明显倾斜几张。\n");
    cv::namedWindow("calib", cv::WINDOW_NORMAL);

    for (;;)
    {
        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) break;

        calib::View view;
        const bool found = detector.detect(frame, "camera_" + std::to_string(views.size()), view);
        cv::Mat display = frame.clone();
        if (found)
        {
            cv::drawChessboardCorners(display, opt.board.patternSize(), view.corners, true);
        }

        const std::string hud =
            "captured " + std::to_string(views.size()) +
            (found ? "  [board OK]  SPACE = capture" : "  [no board]  move the board") +
            "   q = quit";
        cv::putText(display, hud, cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX,
                    0.7, cv::Scalar(0, 255, 255), 2);
        cv::imshow("calib", display);

        const int key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
        if ((key == ' ' || key == 13) && found &&
            static_cast<int>(views.size()) < opt.max_views)
        {
            if (!views.empty() && calib::cornerShift(views.back(), view) < opt.min_move_px)
            {
                std::printf("  这一张和上一张太像，换个角度或位置再拍\n");
            }
            else
            {
                std::printf("  [收录] 第 %zu 张\n", views.size() + 1);
                if (views.empty()) preview_image = frame.clone();
                views.push_back(std::move(view));
            }
        }
    }

    cap.release();
    cv::destroyWindow("calib");
    return true;
}


// ---------- 输入源 4：零素材合成自检 ----------
bool collectSynthetic(const calib::Options& opt, const calib::Intrinsics& ref,
                      std::vector<calib::View>& views, cv::Mat& preview_image)
{
    cv::Size size = ref.size;
    if (size.width <= 0 || size.height <= 0) size = cv::Size(1440, 1080);

    views = calib::makeSyntheticViews(opt.board, ref.K, ref.D, size,
                                      opt.synthetic_views, opt.synthetic_noise);
    if (views.empty())
    {
        std::fprintf(stderr, "合成视图失败：参考内参无效或棋盘太大放不进画面\n");
        return false;
    }

    std::printf("合成自检: 用 fixed 版内参当真值，生成 %zu 个不同姿态的虚拟角点观测"
                "（噪声 σ=%.2f px）\n", views.size(), opt.synthetic_noise);
    std::printf("          标定结果应当几乎精确等于参考内参，差异见下面对照表。\n");
    preview_image = calib::drawSyntheticBoard(views.front(), opt.board);
    return true;
}


int makeFlags(const calib::Options& opt)
{
    int flags = 0;
    if (opt.fix_k3)              flags |= cv::CALIB_FIX_K3;
    if (opt.fix_principal_point) flags |= cv::CALIB_FIX_PRINCIPAL_POINT;
    if (opt.zero_tangent_dist)   flags |= cv::CALIB_ZERO_TANGENT_DIST;
    return flags;
}

}   // namespace


int main(int argc, char** argv)
{
    calib::Options opt = calib::parseOptions(argc, argv);

    if (opt.help) { calib::printUsage(argv[0]); return 0; }
    if (!opt.ok)
    {
        std::fprintf(stderr, "参数错误: %s\n\n", opt.error.c_str());
        calib::printUsage(argv[0]);
        return 2;
    }

    // 第一次运行、又没准备图片时，别让用户对着报错发呆，直接跑一遍自检
    std::error_code exists_ec;
    if (argc == 1 && !fs::exists("images", exists_ec))
    {
        opt.source = calib::Source::Synthetic;
        std::printf("没找到 images/ 目录，自动进入 --synthetic 零素材自检模式。\n\n");
    }

    try
    {
        fs::create_directories(opt.output);

        calib::Intrinsics reference;
        if (!calib::loadIntrinsics(opt.reference, reference))
        {
            reference = calib::fixedFallback();
            std::printf("[提示] 没读到 %s，改用 fixed 版内置内参作为对照基准。\n",
                        opt.reference.c_str());
        }
        std::printf("对照基准: %s\n\n", reference.name.c_str());

        std::vector<calib::View> views;
        cv::Mat preview_image;
        calib::BoardDetector detector(opt.board,
            opt.preview || opt.source == calib::Source::Camera);

        bool collected = false;
        switch (opt.source)
        {
            case calib::Source::Images:    collected = collectImages(opt, detector, views, preview_image);    break;
            case calib::Source::Video:     collected = collectVideo(opt, detector, views, preview_image);     break;
            case calib::Source::Camera:    collected = collectCamera(opt, detector, views, preview_image);    break;
            case calib::Source::Synthetic: collected = collectSynthetic(opt, reference, views, preview_image); break;
        }
        if (!collected || views.empty()) return 1;

        // OpenCV 要求所有视图分辨率一致；混进来的其它尺寸直接丢弃并提示
        const cv::Size size0 = views.front().image_size;
        std::vector<calib::View> consistent;
        for (auto& v : views)
        {
            if (v.image_size == size0) consistent.push_back(std::move(v));
            else std::printf("  [跳过] %s（分辨率 %dx%d 与第一张 %dx%d 不一致）\n",
                             v.label.c_str(), v.image_size.width, v.image_size.height,
                             size0.width, size0.height);
        }
        views = std::move(consistent);
        if (static_cast<int>(views.size()) > opt.max_views) views.resize(opt.max_views);

        if (static_cast<int>(views.size()) < opt.min_views)
        {
            std::fprintf(stderr,
                "\n有效视图只有 %zu 张，少于 --min-views %d。\n"
                "请补拍不同角度/位置的棋盘照片（倾斜的、四角的都要有），\n"
                "或临时用 --min-views 4 降低门槛先跑通流程。\n",
                views.size(), opt.min_views);
            return 1;
        }

        const calib::CalibrationResult result =
            calib::Calibrator::run(views, opt.board, makeFlags(opt), 3.0, opt.refine_outliers);
        if (!result.ok)
        {
            std::fprintf(stderr, "\n标定失败: %s\n", result.message.c_str());
            return 1;
        }

        const std::string yml     = opt.output + "/calibration.yml";
        const std::string snippet = opt.output + "/camParams_snippet.txt";
        const std::string csv     = opt.output + "/per_view_error.csv";
        const std::string undist  = opt.output + "/undistort_preview.jpg";

        calib::printSummary(result, views);
        if (opt.compare) calib::printComparison(reference, result);

        const bool ok_yml = calib::saveCalibration(yml, result, opt, views);
        calib::saveSnippet(snippet, result);
        calib::savePerViewCsv(csv, result, views);
        if (opt.save_undistort && !preview_image.empty())
            calib::saveUndistortPreview(undist, preview_image, result);
        if (opt.preview || opt.source == calib::Source::Camera)
        {
            const int n = calib::savePreviews(opt.output, views, 12);
            std::printf("\n已保存 %d 张角点检测叠加图到 %s/\n", n, opt.output.c_str());
        }

        std::printf("\n========== 输出 ==========\n");
        std::printf("  %s   %s\n", yml.c_str(),     ok_yml ? "OK" : "写入失败");
        std::printf("  %s   可直接粘贴回 fixed 版的 camParams\n", snippet.c_str());
        std::printf("  %s   逐视图误差 / 倾角 / 棋盘位置\n", csv.c_str());
        if (opt.save_undistort && !preview_image.empty())
            std::printf("  %s   去畸变前后对比\n", undist.c_str());
        std::printf("\n回填 fixed 版：把 %s 里的 camParams 覆盖到\n"
                    "kalman_filter_fixed/include/data_interaction.hpp 的同名结构体即可；\n"
                    "PnP 与 EKF 的代码一行都不用改。\n", snippet.c_str());
        return 0;
    }
    catch (const cv::Exception& e)
    {
        std::fprintf(stderr, "OpenCV 异常: %s\n"
                            "（若是摄像头窗口打不开，请改用 --images / --video / --synthetic）\n",
                     e.what());
        return 1;
    }
    catch (const std::exception& e)
    {
        std::fprintf(stderr, "异常: %s\n", e.what());
        return 1;
    }
}
