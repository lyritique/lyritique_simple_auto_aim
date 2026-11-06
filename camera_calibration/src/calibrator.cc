# include "calibrator.hpp"

# include <algorithm>
# include <cmath>
# include <limits>
# include <numeric>
# include <utility>
# include <vector>

# include <opencv2/calib3d.hpp>


namespace calib
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

struct RawResult
{
    cv::Mat K {}, D {};
    std::vector<cv::Mat> rvecs {}, tvecs {};
    double rms = std::numeric_limits<double>::infinity();
    bool ok = false;
    std::string message {};
};


// 跑一次 cv::calibrateCamera。index 里是要用哪几张视图
RawResult solve(const std::vector<View>& views,
                const std::vector<int>& index,
                const std::vector<cv::Point3f>& object,
                int flags)
{
    RawResult out;
    if (index.size() < 4)
    {
        out.message = "有效视图少于 4 张，标定方程欠定";
        return out;
    }

    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    object_points.reserve(index.size());
    image_points.reserve(index.size());
    for (const int i : index)
    {
        object_points.push_back(object);
        image_points.push_back(views[static_cast<std::size_t>(i)].corners);
    }

    try
    {
        /*
         * 张正友标定：每个姿态先闭式解一个单应 H，用 H 的列正交约束线性解出内参，
         * 再把所有参数（内参 + 畸变 + 每张图的外参）丢进 LM 联合优化，
         * 最小化所有角点的重投影误差平方和。OpenCV 返回的 rms 就是这个值的均方根。
         */
        out.rms = cv::calibrateCamera(
            object_points, image_points, views[static_cast<std::size_t>(index.front())].image_size,
            out.K, out.D, out.rvecs, out.tvecs, flags,
            cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-9));

        out.ok = std::isfinite(out.rms) && !out.K.empty() && out.K.total() == 9
                                        && !out.D.empty()
                                        && out.rvecs.size() == index.size();
        if (!out.ok) out.message = "calibrateCamera 返回了无效结果";
    }
    catch (const cv::Exception& e)
    {
        out.ok = false;
        out.message = std::string("calibrateCamera 异常: ") + e.what();
    }
    return out;
}


// 用标定出来的 K/D 和每张图的外参，把三维点重新投回图像，算每张图的 RMS
double reprojectionRms(const std::vector<cv::Point3f>& object,
                       const std::vector<cv::Point2f>& observed,
                       const cv::Mat& rvec, const cv::Mat& tvec,
                       const cv::Mat& K, const cv::Mat& D)
{
    if (object.size() != observed.size() || object.empty()) return -1.0;

    std::vector<cv::Point2f> projected;
    cv::projectPoints(object, rvec, tvec, K, D, projected);
    if (projected.size() != observed.size()) return -1.0;

    double sq = 0.0;
    for (std::size_t i = 0; i < projected.size(); ++i)
    {
        const cv::Point2f d = projected[i] - observed[i];
        sq += static_cast<double>(d.x) * d.x + static_cast<double>(d.y) * d.y;
    }
    return std::sqrt(sq / static_cast<double>(projected.size()));
}


// 板面法向（R 的第三列）与光轴的夹角。0° = 正对相机，越大说明姿态越斜。
// 一堆全是 0° 的正面板会让 fx/fy 与 tz 强耦合，标定结果不可信。
double tiltDeg(const cv::Mat& rvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    const double nz = R.at<double>(2, 2);
    return std::acos(std::clamp(nz, -1.0, 1.0)) * 180.0 / kPi;
}


double medianOf(std::vector<double> v)
{
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    if (n % 2 == 1) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}


}   // namespace


CalibrationResult Calibrator::run(const std::vector<View>& views,
                                  const BoardSpec& board,
                                  int flags,
                                  double reject_sigma,
                                  bool refine_outliers)
{
    CalibrationResult result;
    const std::size_t n = views.size();
    result.per_view_rms.assign(n, -1.0);
    result.first_pass_rms.assign(n, -1.0);
    result.tilt_deg.assign(n, 0.0);
    result.used.assign(n, false);

    if (n < 4)
    {
        result.message = "有效视图少于 4 张，至少要 4 个不同姿态";
        return result;
    }
    result.image_size = views.front().image_size;

    const std::vector<cv::Point3f> object = board.objectPoints();
    std::vector<int> index(static_cast<int>(n));
    std::iota(index.begin(), index.end(), 0);

    RawResult raw = solve(views, index, object, flags);
    if (!raw.ok)
    {
        result.message = raw.message;
        return result;
    }
    result.passes = 1;

    for (std::size_t j = 0; j < index.size(); ++j)
    {
        const int i = index[j];
        result.first_pass_rms[static_cast<std::size_t>(i)] =
            reprojectionRms(object, views[static_cast<std::size_t>(i)].corners,
                            raw.rvecs[j], raw.tvecs[j], raw.K, raw.D);
    }

    /*
     * 离群视图剔除：
     * 用「中位数 + sigma × 1.4826 × MAD」而不是「均值 + 3σ」，因为一两角点检错的
     * 坏图会把均值和标准差一起拉爆，反而漏掉它（这就是masking effect）。
     * 阈值再兜两个下限（med+0.15 与 0.30 px），避免理想合成数据里 MAD≈0 时
     * 把正常的浮点微小差异全判成离群。
     */
    std::vector<int> kept = index;
    if (refine_outliers && reject_sigma > 0.0 && n >= 8)
    {
        const double med = medianOf(result.first_pass_rms);
        std::vector<double> deviation;
        deviation.reserve(n);
        for (const double v : result.first_pass_rms) deviation.push_back(std::fabs(v - med));
        const double mad = medianOf(deviation);
        const double threshold = std::max({med + reject_sigma * 1.4826 * mad,
                                           med + 0.15, 0.30});

        std::vector<int> candidate;
        for (const int i : index)
        {
            if (result.first_pass_rms[static_cast<std::size_t>(i)] <= threshold)
                candidate.push_back(i);
        }
        // 至少留一半、且不少于 4 张，防止把好数据一并砍光
        const std::size_t min_keep = std::max<std::size_t>(4, (n + 1) / 2);
        if (candidate.size() < index.size() && candidate.size() >= min_keep)
        {
            kept = candidate;
            RawResult second = solve(views, kept, object, flags);
            // 只有当结果没明显变差时才采纳第二轮
            if (second.ok && second.rms <= raw.rms + 0.5)
            {
                raw = std::move(second);
                result.passes = 2;
            }
            else
            {
                kept = index;
            }
        }
    }

    result.K = raw.K;
    result.D = raw.D;
    result.rms = raw.rms;
    result.rvecs = raw.rvecs;
    result.tvecs = raw.tvecs;
    result.used_views = static_cast<int>(kept.size());
    result.ok = true;

    for (std::size_t j = 0; j < kept.size(); ++j)
    {
        const int i = kept[j];
        result.used[static_cast<std::size_t>(i)] = true;
        result.per_view_rms[static_cast<std::size_t>(i)] =
            reprojectionRms(object, views[static_cast<std::size_t>(i)].corners,
                            raw.rvecs[j], raw.tvecs[j], raw.K, raw.D);
        result.tilt_deg[static_cast<std::size_t>(i)] = tiltDeg(raw.rvecs[j]);
    }

    if (result.passes == 2)
        result.message = "已剔除离群视图并重标定";
    else
        result.message = "标定完成";
    return result;
}

}   // namespace calib
