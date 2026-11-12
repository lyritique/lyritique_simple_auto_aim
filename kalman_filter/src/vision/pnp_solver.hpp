# ifndef AUTOAIM_VISION_PNP_SOLVER_HPP
# define AUTOAIM_VISION_PNP_SOLVER_HPP

# include <array>
# include <vector>


# include <opencv2/core.hpp>
# include <opencv2/core/types.hpp>
# include <eigen3/Eigen/Eigen>

# include "core/config.hpp"


/*
pnp 类








*/










namespace pnp {

struct pnpConfig
{
    //小装甲板尺寸
    static constexpr double half_small_height {135 * 0.5};
    static constexpr double half_small_width  {55 * 0.5};

    std::vector<cv::Point3d> small_armorPoint 
    {
        cv::Point3f(0.0f, -half_small_width,  half_small_height),          // 左灯条下
        cv::Point3f(0.0f, -half_small_width, -half_small_height),          // 左灯条上
        cv::Point3f(0.0f,  half_small_width, -half_small_height),          // 右灯条上
        cv::Point3f(0.0f,  half_small_width,  half_small_height),          // 右灯条下
    };

    //大装甲板尺寸
    static constexpr double half_big_height {225 * 0.5};
    static constexpr double half_big_width  {55 * 0.5};

    std::vector<cv::Point3d> big_armorPoint 
    {
        cv::Point3f(0.0f, -half_big_width,  half_big_height),          // 左灯条下
        cv::Point3f(0.0f, -half_big_width, -half_big_height),          // 左灯条上
        cv::Point3f(0.0f,  half_big_width, -half_big_height),          // 右灯条上
        cv::Point3f(0.0f,  half_big_width,  half_big_height),          // 右灯条下
    };

    //video内参   
    // 内参已经从 pnpConfig 搬走，改由 core::IntrinsicsConfig / --calibration 提供。

    //解算初筛阈值
    static const constexpr double max_reproj_err {3.0};                                                     //戴帽子大师
    //解算结果cost参数
    static const constexpr double kyaw {0};
    static const constexpr double cost_threhold {33};


};
    






struct PnPResult
{
    bool valid = false;
    Eigen::Matrix<double, 4, 1> measurement {};
    double err {};
    Eigen::Vector3d normal {Eigen::Vector3d::Zero()};   // 板外法向（相机系）
};



class pnpSolver
{
    public:
    //构造函数（未写）
    // 作用：用外部传入的内参拼出 camera_matrix_ 和畸变系数，不再写死在类里。
    // 参数：intrinsics —— 来自 AppConfig，可能已经由 --calibration 从标定文件加载。
    explicit pnpSolver(const core::IntrinsicsConfig& intrinsics);
    void input (const std::array<cv::Point2f, 4>& image_corners, bool big);
    void easySolver();
    void renewPrevious(double& best_yaw);
    PnPResult getOutput();



    const cv::Mat& cameraMatrix() const noexcept
    {
        return camera_matrix_;
    }

    const cv::Mat& distortionCoeffs() const noexcept
    {
        return distortion_coeffs_;
    }

    private:

    // 把4个角点排成与 armorPoint 模板一一对应的顺序：
    // 绕质心按角度排序得到一个不自交的环
    // 旋转环，使"第一条边"落在 135mm 长边上
    // reverse=true 时反向遍历（用来消除镜像）
    // 这样完全不依赖网络关键点输出的顺序。
    //虽然好像无用还浪费性能
    std::array<cv::Point2f, 4> canonicalize(const std::array<cv::Point2d, 4>& raw, bool reverse) const;

    Eigen::Matrix<double,4,1> output = Eigen::Matrix<double, 4, 1>::Zero();
    bool output_valid_ = false;
    std::array<cv::Point2d,4> corners {};

    pnpConfig config {};
    //存装甲板系下的装甲板，构造默认小的
    std::vector<cv::Point3d> armorPoint {config.small_armorPoint};

    cv::Mat camera_matrix_{};
    cv::Mat distortion_coeffs_;

    double proj_err {};

    // 上一帧可信的yaw法向，用来在两个 IPPE 解之间做连续性判断,不要跳了
    //最无用的，要么是与同帧另一个装甲版算dyaw，要么就是无id无法与上帧的装甲版建立联系
    double previous_yaw {10086};
    Eigen::Vector3d previous_normal {Eigen::Vector3d::Zero()};

    // getOutput()用的缓存
    Eigen::Vector3d last_normal_ {Eigen::Vector3d::Zero()};
    int last_branch_ {-1};
    bool last_big_ {false};


};

}   // namespace pnp

# endif
