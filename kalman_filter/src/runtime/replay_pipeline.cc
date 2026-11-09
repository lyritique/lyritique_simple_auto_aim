# include "runtime/replay_pipeline.hpp"

# include "debug/draw.hpp"
# include "runtime/frame_processor.hpp"
# include "sensors/video_source.hpp"
# include "timing/replay_imu.hpp"

# include <opencv2/imgproc.hpp>
# include <opencv2/videoio.hpp>

# include <filesystem>
# include <fstream>
# include <memory>
# include <utility>

namespace realtime
{
namespace
{

Eigen::Matrix3d toMatrix(const Eigen::Quaterniond& q)
{
    return q.normalized().toRotationMatrix();
}

}   // namespace

ReplayPipeline::ReplayPipeline(core::AppConfig config)
    : config_(std::move(config))
{
}

bool ReplayPipeline::run(std::string* error)
{
    try
    {
        const auto imu_data =
            replay::readImuFile(config_.imu_path);
        if (imu_data.empty())
        {
            if (error) *error = "IMU 数据为空";
            return false;
        }

        std::unique_ptr<camera::CameraSource> source =
            std::make_unique<camera::VideoSource>(
                config_.video_path);
        if (!source->open(error)) return false;

        const camera::SourceInfo source_info = source->info();
        if (source_info.frame_rate <= 0.0)
        {
            if (error) *error = "视频帧率无效";
            return false;
        }

        std::error_code fs_error;
        std::filesystem::create_directories(
            config_.output_directory,
            fs_error);
        if (fs_error)
        {
            if (error)
            {
                *error = "无法创建输出目录: " +
                    fs_error.message();
            }
            return false;
        }

        const std::string video_output =
            config_.output_directory + "/output_video.avi";
        cv::VideoWriter writer(
            video_output,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            source_info.frame_rate,
            cv::Size(source_info.width, source_info.height));
        if (!writer.isOpened())
        {
            if (error) *error = "无法创建输出视频: " + video_output;
            return false;
        }

        std::ofstream csv(
            config_.output_directory + "/data.csv");
        if (!csv)
        {
            if (error) *error = "csv 无法创建";
            return false;
        }
        csv << "time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r\r\n";

        FrameProcessor processor(config_);

        const double video_start_time = imu_data.front().time;
        const auto imu_0 = replay::alignImuToImage(
            imu_data,
            video_start_time,
            config_.timing.replay_imu_offset_seconds);
        const Eigen::Matrix3d R_0 = toMatrix(imu_0.q);

        core::TimestampedFrame source_frame;
        for (;;)
        {
            const auto status = source->read(source_frame, error);
            if (status == core::FrameReadStatus::End) break;
            if (status != core::FrameReadStatus::Ok)
            {
                if (!error || error->empty())
                {
                    if (error) *error = "视频读取失败";
                }
                return false;
            }

            cv::Mat frame = source_frame.bgr;
            const double image_time =
                video_start_time + source_frame.timestamp;
            const auto imu_k = replay::alignImuToImage(
                imu_data,
                image_time,
                config_.timing.replay_imu_offset_seconds);
            const Eigen::Matrix3d R =
                R_0.transpose() * toMatrix(imu_k.q);

            const FrameProcessingResult processing =
                processor.process(source_frame, R);
            draw::drawDetections(frame, processing.detections);

            for (const auto& observation : processing.observations)
            {
                for (const auto& point : observation.detection.corners)
                {
                    cv::circle(
                        frame,
                        point,
                        2,
                        cv::Scalar(0, 255, 255),
                        10);
                }
                for (const auto& point : observation.refined_corners)
                {
                    cv::circle(
                        frame,
                        point,
                        2,
                        cv::Scalar(255, 0, 255),
                        10);
                }
            }

            const auto& output = processing.state;

            if (output.valid)
            {
                draw::drawFourArmorReprojections(
                    frame,
                    output,
                    processor.cameraMatrix(),
                    processor.distortionCoeffs(),
                    false);
            }

            if (!output.valid)
            {
                csv << image_time << "偏离过大" << "\r\n";
            }
            else
            {
                csv << image_time << ',' << output.valid << ',';
                for (int i = 0; i < 9; ++i)
                {
                    csv << output.state(i) << ',';
                }
                for (const double value :
                     processing.reprojection_errors)
                {
                    csv << value << ',';
                }
                csv << "\r\n";
            }

            draw::drawStateText(frame, output);
            writer.write(frame);
        }

        csv.close();
        writer.release();
        source->close();
        return true;
    }
    catch (const std::exception& exception)
    {
        if (error) *error = exception.what();
        return false;
    }
}

}   // namespace realtime
