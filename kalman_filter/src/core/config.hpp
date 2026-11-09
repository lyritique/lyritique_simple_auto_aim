# ifndef AUTOAIM_CORE_CONFIG_HPP
# define AUTOAIM_CORE_CONFIG_HPP

# include <string>

namespace core
{

enum class RunMode
{
    Replay,
    Live
};

struct CameraConfig
{
    unsigned int device_index {};
    int timeout_ms {1000};

    // 0 表示不主动修改，保留相机当前值。
    int width {};
    int height {};
    float frame_rate {};
    float exposure_time_us {};
    float gain {-1.0F};
};

struct DetectorConfig
{
    std::string model_path {"model/Infantry_fp32.onnx"};

    int input_width {640};
    int input_height {480};
    float score_threshold {0.5F};
    float nms_threshold {0.45F};
    int ignore_color {-1};
    bool ignore_purple {true};
    int num_color {4};
    int num_class {9};
};

struct SerialConfig
{
    std::string device {"/dev/ttyUSB0"};
    int baud {115200};
    double communication_latency {0.0};
};

struct TimingConfig
{
    double max_history_seconds {2.0};
    double max_attitude_gap_seconds {0.1};
    double max_attitude_prediction_seconds {0.05};
    double attitude_wait_ms {30.0};
    double replay_imu_offset_seconds {0.009};
};

struct AppConfig
{
    RunMode mode {RunMode::Replay};

    std::string video_path {
        "material/2026-05-20_23-57-31 (1).avi"};
    std::string imu_path {
        "material/2026-05-20_23-57-31.txt"};
    std::string output_directory {"output"};

    CameraConfig camera;
    DetectorConfig detector;
    SerialConfig serial;
    TimingConfig timing;

    double target_max_distance {2.5};
};

}   // namespace core

# endif
