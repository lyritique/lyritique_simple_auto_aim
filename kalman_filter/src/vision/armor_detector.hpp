# ifndef AUTOAIM_ARMOR_DETECTOR_
# define AUTOAIM_ARMOR_DETECTOR_

# include <array>
# include <opencv2/core/types.hpp>
# include <string>
# include <vector>

# include <opencv2/core.hpp>
# include <opencv2/dnn.hpp>

# include "core/config.hpp"

/*
detector 类
输入
图象BGR
输出
armor旋转矩形 图象坐标系
角点 图象坐标系
置信，颜色，是否大装甲板，机器人编号

图像坐标系：左上原点，x向右，y向下

*/








namespace detector {
using DetectorConfig = core::DetectorConfig;

// 原图 -> 网络输入etterbox 变换的参数，做后处理时要把网络坐标还原到原图
struct LetterboxInfo
{
    float scale = 1.0f;   // 等比缩放系数
    int pad_x = 0;        // 左侧填充的像素数
    int pad_y = 0;        // 顶部填充的像素数
};

// 装甲板的检测结果。这里不包含 PnP 位姿
struct ArmorDetection
{
    // 四个关键点顺序固定为：左上、左下、右下、右上
    std::array<cv::Point2f, 4> corners {};

    cv::RotatedRect box {};//四点的外接矩形，供 NMS 和画图使用
    float score = 0.0f;   //类别置信度
    int color = -1;       //0 蓝、1 红、2 白、3 紫
    int number = -1;      //机器人类别编号
    bool big = false;     //是否为大装甲板
};

using ArmorDetections = std::vector<ArmorDetection>;//早知道有这个

class Detector
{
    public:
        //构造时加载模型，后面无需再加载
        explicit Detector(const DetectorConfig& config = DetectorConfig{});

        //综合
        ArmorDetections detect(const cv::Mat& bgr);

        const DetectorConfig& config() const noexcept { return config_; }

    private:
        // letterbox 之后的网络输入，同时保存后处理需要的反变换参数
        struct Blob
        {
            cv::Mat blob;   // NCHW：1x3xHxW，CV_32F，值域 0~1
            LetterboxInfo info;
        };

        //BGR -> letterbox  -> blob
        Blob makeBlob(const cv::Mat& bgr) const;

        //把输出解码
        void decode(const cv::Mat& output, const Blob& input,
                    int image_width, int image_height,
                    ArmorDetections& detections) const;

        //贪心非极大值抑制
        void nms(ArmorDetections& detections, float iou_threshold) const;

        DetectorConfig config_;
        cv::dnn::Net net_;
};

}












# endif
