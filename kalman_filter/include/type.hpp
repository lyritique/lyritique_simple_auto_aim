# ifndef __AUTO_AIM_TYPE__
# define __AUTO_AIM_TYPE__

#include "data_interaction.hpp"
# include <array>
# include <opencv2/core/types.hpp>







namespace autoAim 
{
class armor 
{
    public:
    //最初图像的角点
    std::array<cv::Point2d, 4> corners;

    //

};

}

namespace pnp 
{
struct candidate
{
    double rms {};
    double cost {};
    double tz {};
    double yaw {};
    Eigen::Matrix<double, 4, 1> measurement {};
    Eigen::Vector3d normal {Eigen::Vector3d::Zero()};
    cv::Mat rvec {};
    cv::Mat tvec {};
    int branch {};
    bool inward {};          // n·t > 0：法向背离相机
    bool reverse {};
};


}




# endif