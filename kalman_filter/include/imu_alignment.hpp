# ifndef __IMU__
# define __IMU__

# include <eigen3/Eigen/Eigen>
# include <vector>



/*
frameTime               main   视频帧编号到图像时间
alignImuToImage         imu    图像时间、offset、IMU 插值
interpolateQuat         imu     Slerp 插值


*/

namespace kalfilter 
{


struct imu 
{
    double time;
    Eigen::Quaterniond q;
};

std::vector<imu> readImuTxt(std::string file_address);


kalfilter::imu interpolateQuat(const std::vector<kalfilter::imu>& imu_data,double camTime);

kalfilter::imu alignImuToImage(
    const std::vector<kalfilter::imu>& imu_data,
    double image_time,
    double offset = 0.009);
}


# endif
