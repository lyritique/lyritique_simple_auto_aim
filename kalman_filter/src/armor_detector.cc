# include "armor_detector.hpp"

# include <algorithm>
# include <cmath>
#include <opencv2/core/types.hpp>
# include <stdexcept>
# include <utility>

# include <opencv2/imgproc.hpp>

namespace detector {
namespace
{
    constexpr int kKeypointCount = 4;
    constexpr int kKeypointValues = kKeypointCount * 2;

    // 当前模型约定：HERO 和 BASE_B 使用大装甲板
    constexpr int kHeroClass = 1;
    constexpr int kBaseBottomClass = 7;
}


Detector::Detector(const DetectorConfig& config)
    : config_(config)
{
    //检查配置对不对
    if (config_.model_path.empty())
    {
        throw std::invalid_argument("Detector: 模型路径不能为空");
    }
    if (config_.input_width <= 0 || config_.input_height <= 0)
    {
        throw std::invalid_argument("Detector: 输入尺寸必须大于 0");
    }
    if (config_.num_color <= 0 || config_.num_class <= 0)
    {
        throw std::invalid_argument("Detector: 颜色数和类别数必须大于 0");
    }
    if (config_.ignore_color < -1 || config_.ignore_color >= config_.num_color)
    {
        throw std::invalid_argument("Detector: ignore_color 超出颜色分支范围");
    }

    //启动model
    try
    {
        net_ = cv::dnn::readNetFromONNX(config_.model_path);
    }
    catch (const cv::Exception& e)
    {
        throw std::runtime_error("Detector: 加载 ONNX 模型失败: " +
                                 config_.model_path + "\n" + e.what());
    }

    if (net_.empty())
    {
        throw std::runtime_error("Detector: ONNX 模型为空: " + config_.model_path);
    }
}


Detector::Blob Detector::makeBlob(const cv::Mat& bgr) const
{
    Blob input;                                                                                                        //没人觉得在这个函数了应该叫output吗

    //宽、高方向各算一个缩放比例，取较小的那个，不用头脑风暴长宽比比较
    const float scale_x = static_cast<float>(config_.input_width) / bgr.cols;
    const float scale_y = static_cast<float>(config_.input_height) / bgr.rows;
    input.info.scale = std::min(scale_x, scale_y);

    const int resized_width = std::max(
        1, std::min(config_.input_width,
        static_cast<int>(std::lround(bgr.cols * input.info.scale))));
    const int resized_height = std::max(
        1, std::min(config_.input_height,
        static_cast<int>(std::lround(bgr.rows * input.info.scale))));

    input.info.pad_x = (config_.input_width - resized_width) / 2;
    input.info.pad_y = (config_.input_height - resized_height) / 2;

    //仿射变换
    cv::Mat resized;
    //x' = s * x + tx
    //y' = s * y + ty

    /*
    //有resize函数为什么要手敲代码
    const float tx = (config_.input_width  - bgr.cols * input.info.scale) * 0.5f;
    const float ty = (config_.input_height - bgr.rows * input.info.scale) * 0.5f;

    cv::Mat M = (cv::Mat_<float>(2, 3) <<
    input.info.scale, 0.f,              tx,
    0.f,              input.info.scale, ty);

    cv::warpAffine(
    bgr,
    resized,
    M,
    cv::Size(config_.input_width, config_.input_height),
    cv::INTER_LINEAR,
    cv::BORDER_CONSTANT,
    cv::Scalar(114, 114, 114)); 
    */
    
    cv::resize(bgr, resized, cv::Size(resized_width, resized_height),
               0.0, 0.0, cv::INTER_LINEAR);

    //使用与训练一致的灰边填充
    cv::Mat canvas;
    cv::copyMakeBorder(
        resized, canvas,
        input.info.pad_y, config_.input_height - resized_height - input.info.pad_y,
        input.info.pad_x, config_.input_width - resized_width - input.info.pad_x,
        cv::BORDER_CONSTANT, cv::Scalar(124, 124, 124));

    //OpenCV 的 BGR 转成模型训练时使用的 RGB
    //输出布局为 NCHW、CV_32F，并在这里完成 1/255 归一化
    cv::dnn::blobFromImage(
        canvas, input.blob, 1.0 / 255.0,
        cv::Size(config_.input_width, config_.input_height),
        cv::Scalar(), true, false, CV_32F);

    return input;
}


ArmorDetections Detector::detect(const cv::Mat& bgr)
{
    ArmorDetections detections;
    if (bgr.empty()) return detections;

    if (bgr.type() != CV_8UC3)
    {
        throw std::invalid_argument("Detector::detect 需要 CV_8UC3 的 BGR 图像");
    }

    const Blob input = makeBlob(bgr);
    if (input.blob.empty() || input.info.scale <= 0.0f) return detections;

    cv::Mat output;
    try
    {
        net_.setInput(input.blob);
        output = net_.forward();  // 当前模型输出形状为 [1, 21, 6300]
    }
    catch (const cv::Exception& e)
    {
        throw std::runtime_error(
            std::string("Detector: YOLO 前向推理失败: ") + e.what());
    }

    decode(output, input, bgr.cols, bgr.rows, detections);
    nms(detections, config_.nms_threshold);
    return detections;
}


void Detector::decode(const cv::Mat& output, const Blob& input, int image_width,
                      int image_height, ArmorDetections& detections) const
{
    const int expected_channels =
        config_.num_color + config_.num_class + kKeypointValues;

    if (output.dims != 3 || output.size[0] != 1 ||
        output.size[1] != expected_channels)
    {
        const int actual_channels =
            output.dims == 3 ? output.size[1] : -1;
        throw std::runtime_error(
            "Detector: 模型输出形状错误，期望通道数 " +
            std::to_string(expected_channels) + "，实际 " +
            std::to_string(actual_channels));
    }

    const int candidate_count = output.size[2];
    const int class_offset = config_.num_color;
    const int keypoint_offset = config_.num_color + config_.num_class;

    //feature(f, c) = data[c + f * candidate_count]
    //用feature()指针读取
    /*const cv::Mat flat = output.reshape(1, 1);
    const float* data = flat.ptr<float>(0);*/
    const float* data = output.ptr<float>(0);
    const auto feature = [&](int channel, int candidate)
    {
        return data[candidate + channel * candidate_count];
    };

    ArmorDetections candidates;
    candidates.reserve(static_cast<std::size_t>(candidate_count*0.1));

    for (int candidate {}; candidate < candidate_count; ++candidate)
    {
        //类别分支取最大值，当作当前候选的类别和置信度
        int number {};
        float best_score = feature(class_offset, candidate);
        //打擂台
        for (int cls = 1; cls < config_.num_class; ++cls)
        {
            const float score = feature(class_offset + cls, candidate);
            if (score > best_score)
            {
                best_score = score;
                number = cls;
            }
        }
        //剔除置信不够的
        if (best_score < config_.score_threshold) continue;

        //从颜色行中取最大值
        int color {};
        float best_color_score = feature(0, candidate);
        for (int current_color = 1; current_color < config_.num_color; ++current_color)
        {
            const float score = feature(current_color, candidate);
            if (score > best_color_score)
            {
                best_color_score = score;
                color = current_color;
            }
        }
        //去除紫色和标明去除的颜色
        if (config_.ignore_purple && color == 3) continue;
        if (config_.ignore_color >= 0 && color == config_.ignore_color) continue;

        //四个关键点从 letterbox 坐标还原到原图坐标
        ArmorDetection detection;
        bool valid = true;
        for (int keypoint {}; keypoint < kKeypointCount; ++keypoint)
        {
            const float x = feature(keypoint_offset + 2 * keypoint, candidate);
            const float y = feature(keypoint_offset + 2 * keypoint + 1, candidate);
            if (!std::isfinite(x) || !std::isfinite(y))
            {
                valid = false;
                break;
            }

            detection.corners[keypoint].x =
                (x - static_cast<float>(input.info.pad_x)) / input.info.scale;
            detection.corners[keypoint].y =
                (y - static_cast<float>(input.info.pad_y)) / input.info.scale;

            // 任意关键点越界，都说明这个候选不完整，直接丢弃
            if (detection.corners[keypoint].x < 0.0f ||
                detection.corners[keypoint].y < 0.0f ||
                detection.corners[keypoint].x >= static_cast<float>(image_width) ||
                detection.corners[keypoint].y >= static_cast<float>(image_height))
            {
                valid = false;
                break;
            }
        }
        if (!valid) continue;

        //生成斜矩形框，用来nms和center,height,width
        //不要用它的点。由于angle不定，点的顺序无法保证
        detection.box = cv::minAreaRect(detection.corners);   // 成员改成 cv::RotatedRect rrect;

        if (detection.box.size.height <= 0.0f || detection.box.size.width <= 0.0f) continue;

        //包装
        detection.score = best_score;
        detection.color = color;
        detection.number = number;
        detection.big = (number == kHeroClass || number == kBaseBottomClass);
        candidates.push_back(detection);
    }

    detections = std::move(candidates);//整体替换，不用写清除
}


void Detector::nms(ArmorDetections& detections, float iou_threshold) const
{
    if (detections.empty()) return;

    // 先按分数从高到低排序，NMS 只保留高分框
    std::sort(
        detections.begin(), detections.end(),
        [](const ArmorDetection& left, const ArmorDetection& right)
        {
            return left.score > right.score;
        });

    std::vector<cv::RotatedRect> boxes;
    std::vector<float> scores;
    boxes.reserve(detections.size());
    scores.reserve(detections.size());

    for (const ArmorDetection& detection : detections)
    {
        boxes.push_back(detection.box);
        scores.push_back(detection.score);
    }

    std::vector<int> kept_indices;//此乃保留框的下标

    //此处可以距离初筛优化
    //TODO:
    cv::dnn::NMSBoxes(
        boxes, scores, config_.score_threshold, iou_threshold, kept_indices);

    ArmorDetections kept;
    kept.reserve(kept_indices.size());
    for (int index : kept_indices)
    {
        kept.push_back(std::move(detections[index]));
    }
    detections = std::move(kept);
}
}//namespace detector