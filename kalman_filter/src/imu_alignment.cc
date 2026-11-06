# include "imu_alignment.hpp"
# include <eigen3/Eigen/src/Core/Matrix.h>
# include <fstream>
# include <eigen3/Eigen/Dense>
# include <vector>
# include <sstream>
# include <iostream>



namespace kalfilter {



//读取imu数据，直接读出一个imu数据的组
std::vector<kalfilter::imu> readImuTxt(std::string file_address)
{
    std::ifstream fin(file_address);
    //判断是否打开成功
    if (!fin.is_open())
    {
        throw std::runtime_error("打不开 IMU 文件: " + file_address);
        //std::cerr << "无法打开文件\n";
    }

    //逐行读取txt内容，直到getline读取错误
    std::vector<kalfilter::imu> imu_data;

    std::string line;
    int lineNo {};
    while (std::getline(fin,line))
    {
        ++lineNo;
        if (line.empty()) continue;

        std::istringstream iss(line);
        kalfilter::imu r;
        if (iss >> r.time >> r.q.w() >> r.q.x() >> r.q.y() >> r.q.z()) {
            imu_data.push_back(r);
        } else {
            std::cerr << "第 " << lineNo << " 行格式不符，跳过\n";
        }

    }

    
    return imu_data;

}

//由于暂时没有imu数据表，同时offs是一个固定值，就写一个弱智函数
kalfilter::imu alignImuToImage(
    const std::vector<kalfilter::imu>& imu_data,
    double image_time,
    double offset)
{
    return interpolateQuat(imu_data, image_time - offset);
}


//现在要实现imu向相机时刻对齐
//给定一个相机时刻，要要找到对应的imu或相邻的imu在Slerp插值

kalfilter::imu interpolateQuat(const std::vector<kalfilter::imu>& imu_data,double camTime)
{
    //用lowerbound找到第一个camtime小于time
    auto it = std::lower_bound(imu_data.begin(), imu_data.end(), camTime,
                               [](const kalfilter::imu& s, double t) {
                                   return s.time < t;
                               });

    //边界处理
    if (it == imu_data.begin()) {
        // camTime <= 第一帧
        return kalfilter::imu{ imu_data.front().time, imu_data.front().q };
    }
    if (it == imu_data.end()) {
        // camTime >= 最后一帧
        return kalfilter::imu{ imu_data.back().time, imu_data.back().q };
    }

    //用指针记录相邻的imu
    const kalfilter::imu& next = *it;
    const kalfilter::imu& prev = *std::prev(it);

    // SLERP（Eigen）,
    double dt = next.time - prev.time;
    double alpha = (camTime - prev.time) / dt;   

    Eigen::Quaterniond q = prev.q.slerp(alpha, next.q);
    q.normalize();

    return kalfilter::imu{ camTime, q };
}


}//namespace kalfilter




