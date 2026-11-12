# include "timing/replay_imu.hpp"

# include <algorithm>
# include <fstream>
# include <iostream>
# include <iterator>
# include <sstream>
# include <stdexcept>

namespace replay
{

// 作用：把离线 IMU 日志读成内存里的样本表。实现：打开文件、逐行五列解析，成功就收、失败就跳过并计数行号。
std::vector<ImuSample> readImuFile(const std::string& path)
{
    // 作用：打开文件，后事都靠它。实现：ifstream 构造后检查 is_open。
    std::ifstream input(path);
    // 作用：文件都不开就别装作能读，直接报错。实现：抛 runtime_error，带路径方便查。
    if (!input.is_open())
    {
        throw std::runtime_error("打不开 IMU 文件: " + path);
    }

    // 作用：样本仓库，最后整包返回。实现：vector 预先不知道大小，就地 push_back。
    std::vector<ImuSample> samples;
    // 作用：当前行文本的临时座位。实现：getline 复用同一个 string。
    std::string line;
    // 作用：行号计数器，报错时能指到具体哪一行。实现：每读一行自增。
    int line_number = 0;
    // 作用：逐行扫完整个文件。实现：getline 循环，空行直接跳过。
    while (std::getline(input, line))
    {
        ++line_number;
        if (line.empty()) continue;

        // 作用：把这一行当成数据流，逐列取值。实现：istringstream 包一层，用 >> 依次读。
        std::istringstream stream(line);
        ImuSample sample;
        // 作用：五列——时间加四元数四个分量——全读到才算有效样本。实现：链式 >> 的返回值直接当布尔判断。
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
            // 作用：坏行不致命，打个招呼继续读。实现：cerr 输出行号，然后进下一轮。
            std::cerr << "第 " << line_number
                      << " 行格式不符，跳过\n";
        }
    }
    return samples;
}

// 作用：在 IMU 时间轴上插出 image_time 对应的姿态。实现：二分定位邻帧，按比例 SLERP；落在两端外侧就贴边返回。
ImuSample interpolateImu(
    const std::vector<ImuSample>& samples,
    double image_time)
{
    // 作用：空序列没法插，连端点都没有。实现：empty 检查后抛异常，别返回垃圾数据。
    if (samples.empty())
    {
        throw std::runtime_error("IMU 数据为空");
    }

    // 作用：找到第一个时间不小于 image_time 的样本，作为右邻居。实现：lower_bound 配比较 lambda，序列有序所以 O(log n)。
    const auto next = std::lower_bound(
        samples.begin(),
        samples.end(),
        image_time,
        [](const ImuSample& sample, double time)
        {
            return sample.time < time;
        });

    // 作用：目标比所有样本都早，插不出来只能贴左边界。实现：next 是 begin 就返回首样本。
    if (next == samples.begin())
    {
        return samples.front();
    }
    // 作用：目标比所有样本都晚，同理贴右边界。实现：next 是 end 就返回末样本。
    if (next == samples.end())
    {
        return samples.back();
    }

    // 作用：拿到左邻居和区间长度。实现：prev 回退一格，右时间减左时间。
    const auto previous = std::prev(next);
    const double dt = next->time - previous->time;
    // 作用：时间戳重复会让除法爆掉，索性返回左样本。实现：dt 非正就早退。
    if (dt <= 0.0)
    {
        return *previous;
    }

    // 作用：算插值比例，并夹在 [0,1] 内防止浮点越界。实现：先算归一化位置，再 std::clamp。
    const double alpha =
        std::clamp((image_time - previous->time) / dt, 0.0, 1.0);
    // 作用：姿态沿球面最短路径过渡。实现：Eigen 的 slerp，之后 normalize 收尾。
    Eigen::Quaterniond q = previous->q.slerp(alpha, next->q);
    q.normalize();
    // 作用：返回插值结果，时间用请求时刻而不是端点。实现：聚合初始化一个新的 ImuSample。
    return ImuSample {image_time, q};
}

// 作用：补偿 IMU 与相机之间的固定时间差后做对齐。实现：把图像时间往前挪 offset 秒，丢给插值。
ImuSample alignImuToImage(
    const std::vector<ImuSample>& samples,
    double image_time,
    double offset_seconds)
{
    return interpolateImu(samples, image_time - offset_seconds);
}

}   // namespace replay
