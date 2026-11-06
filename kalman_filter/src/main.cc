# include "draw.hpp"
# include "kalman_filter_task.hpp"
# include "armor_detector.hpp"
# include "imu_alignment.hpp"
# include "data_interaction.hpp"

# include <cmath>
# include <opencv2/videoio.hpp>
# include <string_view>
# include <fstream>
# include <filesystem>
# include <vector>
# include <opencv2/imgproc.hpp>
# include <limits>
# include <eigen3/Eigen/Eigen>
# include <iostream>

namespace {
/*
void FrametoKFresult(const cv::Mat& frame,
detector::Detector& detector,pnp::pnpSolver& pnp, kalfilter::KalmanFilter& ekf)
{
    auto detections = detector.detect(frame);

        for (auto detection:detections)
        {
            //pnp
            pnp::PnPResult pose {};
            pnp.input(detection.corners, detection.big);
            pnp.easySolver();
            pose = pnp.getOutput();

            //这里写检测

            //ekf的update
            kalfilter::armerState efk_input{};
            efk_input.measurement = std::move(pose.measurement);
            
        }

}
*/

void computeYPD (Eigen::Matrix<double,4,1>& measurement,kalfilter::cam_ypd& ypd)//x,y,z,yaw
{
    /*
    azi  = atan2(x, z)
    ele  = atan2(y, sqrt(x² + z²))
    dist = sqrt(x² + y² + z²)
    */
    ypd.azi = std::atan2(measurement[0],measurement[2]);
    ypd.ele = std::atan2(measurement[1],
        std::hypot(measurement[0], measurement[2]));
    ypd.dist = std::sqrt(measurement[0]*measurement[0] +
        measurement[1]*measurement[1] +
        measurement[2]*measurement[2]);

}

/*

std::array<cv::Point2f, 4> reorderArmorCorners(
    const std::array<cv::Point2f, 4>& corners)
{
    // 检测器：TL, BL, BR, TR
    // PnP 模板：BL, TL, TR, BR
    return {
        corners[1],  // BL
        corners[0],  // TL
        corners[3],  // TR
        corners[2],  // BR
    };
}

*/
//四元数 归一化 + 变旋转矩阵
Eigen::Matrix3d toMatrix(const Eigen::Quaterniond& q)
{
    Eigen::Quaterniond qn = q.normalized();   // 先归一化，slerp 后数值可能有微小漂移
    return qn.toRotationMatrix();             // 四元数 -> 3x3 旋转矩阵
}


std::array<cv::Point2f, 4> refineArmorCorners(
    const cv::Mat& gray,
    const std::array<cv::Point2f, 4>& input,
    float max_shift = 3.0f,
    cv::Size window_size = cv::Size(5, 5))
{
    std::vector<cv::Point2f> refined(input.begin(), input.end());
    const std::vector<cv::Point2f> original = refined;

    cv::cornerSubPix(
        gray,
        refined,
        window_size,       //搜索窗口
        cv::Size(-1, -1),     // 不使用零区域
        cv::TermCriteria(
            cv::TermCriteria::EPS | cv::TermCriteria::COUNT,
            20,
            0.01));

    std::array<cv::Point2f, 4> result {};

    for (std::size_t i = 0; i < refined.size(); ++i)
    {
        const cv::Point2f delta = refined[i] - original[i];

        const bool inside =
            refined[i].x >= 1.5f &&
            refined[i].y >= 1.5f &&
            refined[i].x < gray.cols - 1.5f &&
            refined[i].y < gray.rows - 1.5f;

        const bool reasonable_shift =
            std::isfinite(refined[i].x) &&
            std::isfinite(refined[i].y) &&
            cv::norm(delta) <= max_shift;

        // 精修不可靠时保留网络原始角点
        result[i] = inside && reasonable_shift
            ? refined[i]
            : original[i];
    }

    return result;
}


}//namespace


int main(int argc, char** argv)
{
    
    //检测命令行输入,理论是 ./kalman_filter [video] [imu_txt] [output_dir]
    //设置默认值
    std::string imu_path = "material/2026-05-20_23-57-31.txt";
    std::string output_path = "output";
    std::string video_path = "material/2026-05-20_23-57-31 (1).avi";

    {
    //输入的参数覆盖默认值
    std::vector<std::string> args(argv + 1, argv + argc);
    //需要检测输入字符串末尾是否有对应后缀
    constexpr std::string_view formEnds[] {".mp4",".avi",".mkv"};

    if (argc > 3 && args[1].ends_with(".txt"))imu_path = args[1];
    if (argc > 2 ) for (auto formEnd :formEnds) if (args[0].ends_with(formEnd))video_path = args[0];
    //千层饼。先是有这个参数，再将其依次与视频后缀做判断
    if (argc > 4) output_path = args[3];
    }
    
    //创建输出目录
    std::filesystem::create_directories(output_path);

    //几个部件全都构建初始化了
    auto imu_data = kalfilter::readImuTxt(imu_path);//内涵错误报错
    if (imu_data.empty()) throw std::runtime_error("IMU 数据为空");
    detector::Detector detector;
    pnp::pnpSolver pnp;
    kalfilter::KalmanFilter ekf;//都使用默认值

    //video的读写
    cv::VideoCapture cap(video_path);//写这个就要小心了
    if (!cap.isOpened()) throw std::runtime_error("读不了视频文件: " + video_path);
    //获得视频相关信息fps与总帧数
    double fps {cap.get(cv::CAP_PROP_FPS)};
    double frame_number {cap.get(cv::CAP_PROP_FRAME_COUNT)};
    if (fps <= 0 ||fps > 1000) throw std::runtime_error("视频有问题");
    //读第一帧，拿cv::size
    cv::Mat frame;
    cap >> frame;
    if (frame.empty())  throw std::runtime_error("第一帧读不了。 " + video_path) ;
    cv::Size frameSize = frame.size(); 
    int width  = frame.cols;
    int height = frame.rows;
    
    
    //用上面的数据构建输出video的writer
    cv::VideoWriter writer(output_path + "/output_video.avi", 
        cv::VideoWriter::fourcc('M','J','P','G'),fps, frameSize);
    if (!writer.isOpened()) throw std::runtime_error("无法创建输出视频");

    
    //CSV的创建
    std::ofstream csv (output_path + "/data.csv");
    if (!csv) throw std::runtime_error("csv无法创建。");
    csv << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r\r\n";
    
    //重设为第一帧
    cap.set(cv::CAP_PROP_POS_FRAMES, 0);

    // 世界系 = 第 0 帧相机系，用 IMU 相对姿态把云台转动补掉
    std::vector<Eigen::Matrix3d> R_wc(static_cast<std::size_t>(frame_number), Eigen::Matrix3d::Identity());
    

    //吸收教训，在循环内初始化
    bool ekf_initialized = false;
    //记一下重投影误差
    std::vector<double> errs {};

    //相信imu的总时间（22.5s）还是视频的总时间（17s），是个问题
    //我认为1秒帧数不是固定的，转到视频时变成30FPS了
    //所有视频时间不可信，要重新计算
    const double video_start_time = imu_data.front().time;
    double previous_image_time {};
    bool has_previous_time = false;

    for (int k = 0; k < frame_number; ++k)
    {
        const double image_time = video_start_time + k / fps;
        const auto imu_k = kalfilter::alignImuToImage(imu_data, image_time);  // 内含 9ms offset
        const Eigen::Matrix3d R_k = toMatrix(imu_k.q);
        if (k == 0) continue;                                                // 第 0 帧保持单位阵
        const auto imu_0 = kalfilter::alignImuToImage(imu_data, video_start_time);
        R_wc[k] = toMatrix(imu_0.q).transpose() * R_k;   // R0ᵀ · Rk
    }

    for (int frame_index {};frame_index < frame_number;++frame_index)
    {
        //处理时间
        const double image_time {video_start_time + frame_index / fps};
        const double dt {image_time-previous_image_time};
        previous_image_time = image_time;
        
        const kalfilter::imu cam_imu {kalfilter::alignImuToImage (imu_data,image_time)};

        if (has_previous_time && ekf_initialized) ekf.predict(dt);
        has_previous_time = true;

        cap.read(frame);

        const Eigen::Matrix3d& R = R_wc[frame_index];

        //灰度图，用来精修角点
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        

        auto detections = detector.detect(frame);
        kalfilter::cam_ypd ypd {};

        //draw装甲板
        draw::drawDetections(frame, detections);

        for (auto detection:detections)
        {
            const double edge_length {cv::norm(detection.corners[0] - detection.corners[1])};
            const int half_window {std::clamp(static_cast<int>(std::lround(edge_length * 0.08f)), 2, 5)};



            const auto ordered {std::move(detection.corners)};

            const auto refined {refineArmorCorners(gray, ordered, 2.0f,cv::Size(half_window, half_window))};

            // 原始角点
            pnp.input(detection.corners, detection.big);
            pnp.easySolver();
            const pnp::PnPResult raw_pose = pnp.getOutput();

            // 精修角点
            pnp.input(refined, detection.big);
            pnp.easySolver();
            const pnp::PnPResult refined_pose = pnp.getOutput();

            bool use_refined = refined_pose.valid &&(!raw_pose.valid ||refined_pose.err + 0.03 < raw_pose.err);

            pnp::PnPResult pose =use_refined ? refined_pose : raw_pose;

            if (!pose.valid) continue;

            //画两种角点
            for (const auto& p : detection.corners)
            cv::circle(frame, p, 2, cv::Scalar(0, 255, 255), 10);

            for (const auto& p : refined)
            cv::circle(frame, p, 2, cv::Scalar(255, 0, 255), 10);
                        
            errs.push_back(pose.err);

            constexpr double kTargetMaxDistance = 2.5;  // 当前视频专用

            computeYPD(pose.measurement, ypd);

            if (ypd.dist > kTargetMaxDistance)
            {
                // 远处的背景机器车，不送 EKF
                continue;
            }



            // 世界系转换
            const Eigen::Vector3d p_cam(pose.measurement(0), pose.measurement(1),
                                        pose.measurement(2));
            const Eigen::Vector3d n_cam = pose.normal;
            const Eigen::Vector3d p_w = R * p_cam;
            const Eigen::Vector3d n_w = R * n_cam;

            //ekf的update
            
            kalfilter::armerState efk_input{};

            efk_input.measurement << p_w.x(), p_w.y(), p_w.z(),std::atan2(n_w.x(), n_w.z());


            if (!ekf_initialized)
            {
                ekf.init(efk_input);
                ekf_initialized = true;
                break;
            }
            ekf.update(efk_input, ypd);
            std::cout << "\n";
        }
        ekf.renewArmorList();

        auto output { ekf.getOutput()};



        

        if (output.valid)
        {
        draw::drawFourArmorReprojections(
        frame,
        output,
        pnp.cameraMatrix(),
        pnp.distortionCoeffs(),
        false);
        }

        //csv输入
        if (!output.valid)csv << image_time << "偏离过大" << "\r\n";//多次偏离应该reset，但还未写
        else 
        {
            csv << image_time  << ','<< output.valid  << ',' ;
            for (int i {}; i < 9; ++i)
            {
                csv <<  output.state(i) << ',';
            }
            for (int i {}; i < errs.size(); ++i)
            {
                csv << errs[i] << ',';
            }
            csv << "\r\n" ;
        }

        errs.clear();

        //这里draw数据
        draw::drawStateText(frame, output);
        writer.write(frame);
    }//循环结束

    //收尾
    csv.close();
    writer.release();
    cap.release();





    /*
    for (int frame_index = 0; cv::readOneFrame(cap, frame); ++frame_index) {
        const double image_time = timelineTime(frame_index);
        const auto camera_imu =interpolateQuat(imu_data, image_time - 0.009);

        auto detections = detector.detect(frame);

        // 1. 选择本车目标，再对候选装甲板做 PnP
        // 2. 只保留 valid、类别/颜色一致、离预测位置合理的观测
        auto measurements = detectionsToMeasurements(detections, pnp);

        if (ekf.initialized()) {
            ekf.predict(image_time - previous_image_time, config);

            // 每帧只 predict 一次；多个可见板可以在 predict 后依次 update
            updateWithAssociatedMeasurements(ekf, measurements, camera_imu, config);
            ekf.renewArmorList();
        } else {
            // 连续若干帧 PnP 稳定后再 init，避免单帧错误初始化
            if (measurementsAreStable(measurements)) {
                ekf.init(measurements.front().armor);
            }
        }

        const auto state = ekf.getOutput(); // 需要新增只读接口
        logToCsv(image_time, state, ekf.covariance());

        drawDetections(frame, detections);
        drawFourArmorReprojections(frame, state, camera_imu);
        drawStateText(frame, state);
        writer.write(frame);

        previous_image_time = image_time;
        has_previous_time = true;
    }

    */
}
