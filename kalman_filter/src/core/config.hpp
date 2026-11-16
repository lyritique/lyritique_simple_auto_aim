#ifndef AUTOAIM_CORE_CONFIG_HPP
#define AUTOAIM_CORE_CONFIG_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace core {

enum class RunMode { Replay, Live };

/*
 * 相机内参。默认值来自第三周作业提供的参考标定，1440×1080。
 * 用 --calibration 指定标定文件后，这里的数值会被文件内容覆盖。
 */
struct IntrinsicsConfig {
  // 标定文件路径；空表示不加载，直接用下面的默认值。
  std::string calibration_path{};

  // 与标定文件 image_width / image_height 对应，用来校验画面尺寸。
  int width{1440};
  int height{1080};

  double fx{2362.2795952460974};
  double fy{2336.1916035376717};
  double cx{679.17513921636282};
  double cy{529.06858905837635};
  // k1, k2, p1, p2, k3；用 vector 是为了兼容 4/5/8 系数。
  std::vector<double> dist{-0.098561592108101559, 0.11173066531932681, -0.0038352945867108721, -0.0020421443354436304,
                           0.0};
};

struct CameraConfig {
  unsigned int device_index{};
  int timeout_ms{1000};

  // 0 表示不主动修改，保留相机当前值。
  int width{};
  int height{};
  float frame_rate{};
  float exposure_time_us{};
  float gain{-1.0F};

  // 时间戳策略：把相机设备时钟映射到主机单调钟。
  // 默认关：这台相机的设备戳按「设定帧率」计数，实际帧率一旦达不到设定值
  // （25 ms 曝光 + 读出开销 ≈ 40 ms，只能跑 24.9 fps），戳就会以每秒几毫秒的速度落后，
  // 跑一分钟偏几秒。收帧时刻自带 1~2 ms 抖动但没有系统偏差，更稳妥。
  // 想要低抖动、且确认相机能跑满设定帧率时，再开 --camera-time-source device。
  bool use_device_timestamp{false};
  // 台架测出的常数偏差，正值表示把帧时间往后挪（µs）。
  double timestamp_offset_us{0.0};
  // 设备戳落在曝光区间的比例：0=曝光开始，0.5=中点，1=结束。
  float exposure_anchor{0.5F};
  // 尝试开启 Chunk-Exposure 拿每帧曝光时间；不支持就自动退回。
  bool chunk_exposure{true};
  // 打印时钟校准结果和每帧时间戳，台架实验用。
  bool timing_log{false};
  // 设备时钟在线校准窗口（秒）。0 表示按 100 帧算；标定 ns/tick 时给几百秒。
  double calib_seconds{0.0};
  // 已知的每 tick 纳秒数。默认 10，对应相机 100 MHz 计数器，
  // 实测已验证：60 fps 下 Δtick=1667091（16.671 ms），20 fps 下 5017847（50.178 ms）。
  // 只有在换相机或要重新标定时才改，填了就不再做斜率拟合。
  double tick_ns{10.0};
};

// 场地曝光预设：曝光时间、增益、帧率三者必须一起配，改一个不改另一个会互相打架。
enum class ExposurePreset {
  Dark,    // 暗实验室
  Indoor,  // 室内正常照明
  Bright   // 强光 / 补光灯直射
};

// 作用：把预设展开成相机参数。实现：三组固定值直接写回 CameraConfig，返回假表示取值不合法。
inline bool applyExposurePreset(ExposurePreset preset, CameraConfig& camera) {
  switch (preset) {
    case ExposurePreset::Dark:
      camera.exposure_time_us = 50000.0F;  // 50 ms → 20 fps 上限
      camera.gain = 0.0F;
      camera.frame_rate = 20.0F;
      return true;
    case ExposurePreset::Indoor:
      camera.exposure_time_us = 25000.0F;  // 25 ms → 40 fps 上限
      camera.gain = 0.0F;
      camera.frame_rate = 40.0F;
      return true;
    case ExposurePreset::Bright:
      camera.exposure_time_us = 8000.0F;  // 8 ms → 60 fps
      camera.gain = 0.0F;
      camera.frame_rate = 60.0F;
      return true;
  }
  return false;
}

struct DetectorConfig {
  std::string model_path{"model/Infantry_fp32.onnx"};

  int input_width{640};
  int input_height{480};
  float score_threshold{0.5F};
  float nms_threshold{0.45F};
  int ignore_color{-1};
  bool ignore_purple{true};
  int num_color{4};
  int num_class{9};

  // 推理后端：auto = TensorRT → CUDA → CPU 逐级回退；trt/cuda 只降不升；cpu 强制 CPU。
  std::string backend{"auto"};
  // TensorRT FP16，可用 --no-fp16 关闭。
  bool fp16{true};
  // TensorRT engine / timing 缓存目录；空字符串表示禁用缓存。
  std::string engine_cache_dir{"model/trt_cache"};
  int trt_workspace_mb{512};
  int trt_builder_opt_level{1};
  // 小于该节点数的子图不交给 TensorRT，避免切出很多小 engine 拖慢构建。
  int trt_min_subgraph_size{5};
  // 打印 ORT / TensorRT 初始化日志，排查节点分配用。
  bool detector_verbose{false};
};

struct SerialConfig {
  std::string device{"/dev/ttyACM0"};
  int baud{115200};
  double communication_latency{0.0};
};

// 云台指令。默认只算不发：时间同步和弹道都没验证之前，别让云台自己动。
struct ControlConfig {
  // 真的把 VisionToGimbal 写进串口。不开就只在开了 debug_log 时打印。
  bool send{false};
  // 打印“算出来的指令”和“C 板自己报的姿态”，先用实测数据判断坐标系约定。
  bool debug_log{false};
  // 指令里的 mode 字节，含义由 C 板定义，默认 0。
  uint8_t mode{0};
  // C 板没报弹速时的兜底值，m/s。
  double bullet_speed{25.0};
  // C 板世界系 yaw / pitch 的正方向。实测发现反了就传 -1，不用改代码。
  double yaw_sign{1.0};
  double pitch_sign{1.0};
  // 零位补偿（度）：C 板控制用的 yaw 零位和它四元数世界系的 yaw 不一致时，差值填这里。
  double yaw_offset_deg{0.0};
  double pitch_offset_deg{0.0};
  // 把 EKF 的相机系观测换到 C 板体轴（x 前/y 左/z 上）。关掉只在排查坐标系问题时用。
  bool cam_remap{true};
  // C 板世界系哪根轴朝天：0 = y 轴（x-z 水平），1 = z 轴（x-y 水平，yaw=atan2(y,x)）。
  // 实测确认是 z 朝天（C 板 yaw/pitch = 体轴 +x 的方位角/仰角），默认 1。
  uint8_t up_axis{1};
  // 弹道下坠补偿，未标定前默认关。
  bool compensate_drop{false};
  double gravity{9.8};
};

struct TimingConfig {
  double max_history_seconds{2.0};
  double max_attitude_gap_seconds{0.1};
  double max_attitude_prediction_seconds{0.05};
  // 等姿态的宽限。检测已经先跑了，这里只兜"包刚好在路上"的几毫秒；
  // 等不到就外推，不要再把整帧堵住。
  double attitude_wait_ms{2.0};
  double replay_imu_offset_seconds{0.009};
};

struct AppConfig {
  RunMode mode{RunMode::Replay};

  std::string video_path{"material/2026-05-20_23-57-31 (1).avi"};
  std::string imu_path{"material/2026-05-20_23-57-31.txt"};
  std::string output_directory{"output"};

  CameraConfig camera;
  IntrinsicsConfig intrinsics;
  DetectorConfig detector;
  SerialConfig serial;
  TimingConfig timing;
  ControlConfig control;

  // 无 IMU 模式：不打开串口、不等待云台姿态，世界系固定在初始相机系（假定云台静止）。
  bool imu_enabled{true};

  // 把每块装甲板的原始观测量（YPD）单独落盘，标定 R 用。
  bool log_observations{false};

  // 每 50 帧打印一次各阶段平均耗时，排查性能瓶颈用。
  bool profile{false};

  double target_max_distance{2.5};
};

}  // namespace core

#endif
