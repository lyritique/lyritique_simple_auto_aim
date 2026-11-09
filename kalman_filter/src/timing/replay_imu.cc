# include "timing/replay_imu.hpp"

# include <algorithm>
# include <fstream>
# include <iostream>
# include <iterator>
# include <sstream>
# include <stdexcept>

namespace replay
{

std::vector<ImuSample> readImuFile(const std::string& path)
{
    std::ifstream input(path);
    if (!input.is_open())
    {
        throw std::runtime_error("打不开 IMU 文件: " + path);
    }

    std::vector<ImuSample> samples;
    std::string line;
    int line_number = 0;
    while (std::getline(input, line))
    {
        ++line_number;
        if (line.empty()) continue;

        std::istringstream stream(line);
        ImuSample sample;
        if (stream >> sample.time >>
                sample.q.w() >>
                sample.q.x() >>
                sample.q.y() >>
                sample.q.z())
        {
            samples.push_back(sample);
        }
        else
        {
            std::cerr << "第 " << line_number
                      << " 行格式不符，跳过\n";
        }
    }
    return samples;
}

ImuSample interpolateImu(
    const std::vector<ImuSample>& samples,
    double image_time)
{
    if (samples.empty())
    {
        throw std::runtime_error("IMU 数据为空");
    }

    const auto next = std::lower_bound(
        samples.begin(),
        samples.end(),
        image_time,
        [](const ImuSample& sample, double time)
        {
            return sample.time < time;
        });

    if (next == samples.begin())
    {
        return samples.front();
    }
    if (next == samples.end())
    {
        return samples.back();
    }

    const auto previous = std::prev(next);
    const double dt = next->time - previous->time;
    if (dt <= 0.0)
    {
        return *previous;
    }

    const double alpha =
        std::clamp((image_time - previous->time) / dt, 0.0, 1.0);
    Eigen::Quaterniond q = previous->q.slerp(alpha, next->q);
    q.normalize();
    return ImuSample {image_time, q};
}

ImuSample alignImuToImage(
    const std::vector<ImuSample>& samples,
    double image_time,
    double offset_seconds)
{
    return interpolateImu(samples, image_time - offset_seconds);
}

}   // namespace replay
