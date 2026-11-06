# include "calib_config.hpp"

# include <cctype>
# include <cstdio>
# include <string>
# include <vector>


namespace calib
{

namespace
{

bool toInt(const std::string& s, int& out)
{
    try
    {
        std::size_t pos = 0;
        const int v = std::stoi(s, &pos);
        if (pos != s.size()) return false;
        out = v;
        return true;
    }
    catch (...) { return false; }
}

bool toDouble(const std::string& s, double& out)
{
    try
    {
        std::size_t pos = 0;
        const double v = std::stod(s, &pos);
        if (pos != s.size()) return false;
        out = v;
        return true;
    }
    catch (...) { return false; }
}

}   // namespace


std::vector<cv::Point3f> BoardSpec::objectPoints() const
{
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            points.emplace_back(static_cast<float>(c * square),
                                static_cast<float>(r * square),
                                0.0f);
        }
    }
    return points;
}


void printUsage(const char* argv0)
{
    std::printf(
        "相机标定程序（依托 kalman_filter_fixed 的工程骨架，结果可直接回填其 pnpSolver）\n"
        "\n"
        "用法: %s [图片目录] [选项]\n"
        "\n"
        "输入源（四选一，默认是图片目录 images/）:\n"
        "  --images <dir>        读取目录里的 jpg/png/bmp/tif 棋盘格照片\n"
        "  --video <file>        从视频里抽帧（配合 --stride / --min-move 去重）\n"
        "  --camera [id]         打开摄像头，空格抓一帧、q 或 ESC 结束（需要图形界面）\n"
        "  --synthetic           零素材自检：用 fixed 版内参合成虚拟角点，标定后应还原真值\n"
        "                        不带任何参数且 images/ 不存在时会自动进入这个模式\n"
        "\n"
        "标定板:\n"
        "  --cols <n>            横向内角点数，默认 11（即 12 列格子）\n"
        "  --rows <n>            纵向内角点数，默认 8 （即 9 行格子）\n"
        "  --square <mm>         单个方格边长，默认 25.0\n"
        "\n"
        "模型开关:\n"
        "  --fix-k3              固定三阶径向畸变 k3 = 0（fixed 版给的文件里 k3 就是 0）\n"
        "  --fix-principal       主点固定在图像中心\n"
        "  --zero-tangent        切向畸变 p1 = p2 = 0\n"
        "  --no-refine           不剔除离群视图\n"
        "  --no-compare          不打印与 fixed 版内参的对照\n"
        "\n"
        "采集 / 输出:\n"
        "  --output <dir>        输出目录，默认 output/\n"
        "  --reference <yml>     对照用内参文件，默认 config/reference_intrinsics.yml\n"
        "  --preview             额外保存每张检测图的角点叠加图\n"
        "  --min-views <n>       最少有效视图数，默认 8\n"
        "  --max-views <n>       最多使用的视图数，默认 40\n"
        "  --min-move <px>       视频/摄像头去重阈值，默认 25\n"
        "  --stride <n>          视频抽帧间隔，默认 3\n"
        "  --synthetic-views <n> 合成视图数量，默认 20\n"
        "  --synthetic-noise <px>给合成角点加的高斯噪声，默认 0\n"
        "  -h, --help            显示本帮助\n"
        "\n"
        "示例:\n"
        "  ./build/camera_calibration                         # 无素材自检\n"
        "  ./build/camera_calibration --synthetic --fix-k3    # 自检 + 固定 k3\n"
        "  ./build/camera_calibration --images images --cols 11 --rows 8 --square 25 --preview\n"
        "  ./build/camera_calibration --video material/calib.avi --stride 5\n"
        "  ./build/camera_calibration --camera 0               # 实时采集\n",
        argv0);
}


Options parseOptions(int argc, char** argv)
{
    Options opt;
    bool source_set = false;
    std::vector<std::string> positional;

    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];

        // 取下一个参数作为值；缺值就把错误记下来，最后统一报
        auto needValue = [&](const char* name) -> std::string
        {
            if (i + 1 >= argc)
            {
                opt.ok = false;
                opt.error = std::string("参数 ") + name + " 缺少取值";
                return std::string();
            }
            return std::string(argv[++i]);
        };

        auto setInt = [&](const char* name, int& dst)
        {
            const std::string v = needValue(name);
            if (!opt.ok) return;
            if (!toInt(v, dst))
            {
                opt.ok = false;
                opt.error = std::string("参数 ") + name + " 需要整数，收到: " + v;
            }
        };

        auto setDouble = [&](const char* name, double& dst)
        {
            const std::string v = needValue(name);
            if (!opt.ok) return;
            if (!toDouble(v, dst))
            {
                opt.ok = false;
                opt.error = std::string("参数 ") + name + " 需要数字，收到: " + v;
            }
        };

        if (a == "-h" || a == "--help") { opt.help = true; }
        else if (a == "--images")
        {
            opt.source = Source::Images; source_set = true;
            const std::string v = needValue("--images");
            if (opt.ok) opt.input = v;
        }
        else if (a == "--video")
        {
            opt.source = Source::Video; source_set = true;
            const std::string v = needValue("--video");
            if (opt.ok) opt.input = v;
        }
        else if (a == "--camera")
        {
            // id 可以省略，默认 0；只有下一个参数确实是数字时才吞掉它
            opt.source = Source::Camera; source_set = true;
            if (i + 1 < argc && std::isdigit(static_cast<unsigned char>(argv[i + 1][0])))
            {
                const std::string v = needValue("--camera");
                if (opt.ok && !toInt(v, opt.camera_id))
                {
                    opt.ok = false;
                    opt.error = "--camera 需要摄像头序号，收到: " + v;
                }
            }
        }
        else if (a == "--synthetic") { opt.source = Source::Synthetic; source_set = true; }
        else if (a == "--cols")    { setInt("--cols", opt.board.cols); }
        else if (a == "--rows")    { setInt("--rows", opt.board.rows); }
        else if (a == "--square")  { setDouble("--square", opt.board.square); }
        else if (a == "--output")  { const std::string v = needValue("--output");    if (opt.ok) opt.output = v; }
        else if (a == "--reference"){ const std::string v = needValue("--reference"); if (opt.ok) opt.reference = v; }
        else if (a == "--fix-k3")            { opt.fix_k3 = true; }
        else if (a == "--fix-principal")     { opt.fix_principal_point = true; }
        else if (a == "--zero-tangent")      { opt.zero_tangent_dist = true; }
        else if (a == "--preview")           { opt.preview = true; }
        else if (a == "--no-compare")        { opt.compare = false; }
        else if (a == "--no-refine")         { opt.refine_outliers = false; }
        else if (a == "--min-views")         { setInt("--min-views", opt.min_views); }
        else if (a == "--max-views")         { setInt("--max-views", opt.max_views); }
        else if (a == "--stride")            { setInt("--stride", opt.frame_stride); }
        else if (a == "--synthetic-views")   { setInt("--synthetic-views", opt.synthetic_views); }
        else if (a == "--min-move")          { setDouble("--min-move", opt.min_move_px); }
        else if (a == "--synthetic-noise")   { setDouble("--synthetic-noise", opt.synthetic_noise); }
        else if (a.rfind("--", 0) == 0 || a.rfind("-", 0) == 0)
        {
            opt.ok = false;
            opt.error = "未知参数: " + a;
        }
        else { positional.push_back(a); }
    }

    // 位置参数 = 输入路径。没显式指定输入源时，它就是一个图片目录
    if (!positional.empty())
    {
        opt.input = positional.front();
        if (!source_set) opt.source = Source::Images;
    }

    // ---- 合法性检查：宁可早点报错，也不要让 calibrateCamera 吐一堆莫名其妙的结果 ----
    if (opt.ok && (opt.board.cols < 3 || opt.board.rows < 3))
    {
        opt.ok = false; opt.error = "--cols / --rows 是内角点数，至少要 3";
    }
    if (opt.ok && !(opt.board.square > 0.0))
    {
        opt.ok = false; opt.error = "--square 必须为正数";
    }
    if (opt.ok && opt.min_views < 4)
    {
        opt.ok = false; opt.error = "--min-views 至少 4 张，否则标定方程欠定";
    }
    if (opt.ok && opt.max_views < opt.min_views)
    {
        opt.ok = false; opt.error = "--max-views 不能小于 --min-views";
    }
    if (opt.ok && opt.frame_stride < 1)
    {
        opt.ok = false; opt.error = "--stride 至少为 1";
    }
    if (opt.ok && !(opt.min_move_px >= 0.0))
    {
        opt.ok = false; opt.error = "--min-move 不能为负数";
    }
    if (opt.ok && !(opt.synthetic_noise >= 0.0))
    {
        opt.ok = false; opt.error = "--synthetic-noise 不能为负数";
    }
    if (opt.ok && opt.synthetic_views < 4)
    {
        opt.ok = false; opt.error = "--synthetic-views 至少 4";
    }
    return opt;
}

}   // namespace calib
