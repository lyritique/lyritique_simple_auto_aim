#include "vision/armor_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <utility>

namespace detector {
namespace {
constexpr int kKeypointCount = 4;
constexpr int kKeypointValues = kKeypointCount * 2;

// 当前模型约定：HERO 和 BASE_B 使用大装甲板
constexpr int kHeroClass = 1;
constexpr int kBaseBottomClass = 7;

const char* backendName(Backend backend) {
  switch (backend) {
    case Backend::TensorRt:
      return "TensorRT";
    case Backend::Cuda:
      return "CUDA";
    case Backend::Cpu:
      return "CPU";
  }
  return "Unknown";
}

// 把 CLI 的字符串展开成候选链：只降不升，CPU 永远是最后兜底。
std::vector<Backend> candidateBackends(const std::string& requested) {
  if (requested == "cpu") return {Backend::Cpu};
  if (requested == "cuda") return {Backend::Cuda, Backend::Cpu};
  return {Backend::TensorRt, Backend::Cuda, Backend::Cpu};
}

// ORT 的 provider 列表只取决于安装包，进程内不会变化，缓存一次即可。
bool providerAvailable(const char* name) {
  static const std::vector<std::string> providers = Ort::GetAvailableProviders();
  return std::find(providers.begin(), providers.end(), name) != providers.end();
}

// 每个候选后端都新建一份 SessionOptions，避免失败后带着半套 EP 状态重试。
Ort::SessionOptions makeOptions(Backend backend, const DetectorConfig& config) {
  Ort::SessionOptions options;
  options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
  // 单帧同步推理，多线程只会互相抢核。
  options.SetIntraOpNumThreads(1);
  options.SetInterOpNumThreads(1);
  options.SetLogSeverityLevel(config.detector_verbose ? ORT_LOGGING_LEVEL_VERBOSE : ORT_LOGGING_LEVEL_WARNING);

  const OrtApi& api = Ort::GetApi();

  if (backend == Backend::TensorRt) {
    OrtTensorRTProviderOptionsV2* trt_options = nullptr;
    Ort::ThrowOnError(api.CreateTensorRTProviderOptions(&trt_options));

    const auto release = [&api](OrtTensorRTProviderOptionsV2* pointer) {
      if (pointer != nullptr) {
        api.ReleaseTensorRTProviderOptions(pointer);
      }
    };
    const std::unique_ptr<OrtTensorRTProviderOptionsV2, decltype(release)> guard(trt_options, release);

    // keys 存常量名，values 存字符串本体；value_ptrs 最后统一取地址，
    // 避免把临时 string 的 c_str() 存进 vector。
    std::vector<const char*> keys;
    std::vector<std::string> values;
    const auto add = [&](const char* key, std::string value) {
      keys.push_back(key);
      values.push_back(std::move(value));
    };

    add("device_id", "0");
    add("trt_fp16_enable", config.fp16 ? "1" : "0");
    add("trt_max_workspace_size", std::to_string(static_cast<long long>(config.trt_workspace_mb) * 1024LL * 1024LL));
    add("trt_builder_optimization_level", std::to_string(config.trt_builder_opt_level));
    add("trt_min_subgraph_size", std::to_string(config.trt_min_subgraph_size));
    add("trt_engine_cache_enable", config.engine_cache_dir.empty() ? "0" : "1");

    if (!config.engine_cache_dir.empty()) {
      const std::string model_stem = std::filesystem::path(config.model_path).stem().string();
      add("trt_engine_cache_path", config.engine_cache_dir);
      add("trt_engine_cache_prefix", model_stem + (config.fp16 ? "_fp16" : "_fp32"));
      add("trt_timing_cache_enable", "1");
      add("trt_timing_cache_path", config.engine_cache_dir);
    }
    if (config.detector_verbose) {
      add("trt_dump_subgraphs", "1");
    }

    std::vector<const char*> value_ptrs;
    value_ptrs.reserve(values.size());
    for (const std::string& value : values) {
      value_ptrs.push_back(value.c_str());
    }

    Ort::ThrowOnError(
        api.UpdateTensorRTProviderOptions(trt_options, keys.data(), value_ptrs.data(), static_cast<int>(keys.size())));
    Ort::ThrowOnError(api.SessionOptionsAppendExecutionProvider_TensorRT_V2(options, trt_options));
  }

  // TRT 不支持的算子需要 CUDA 接住，否则会掉到 CPU。
  if (backend == Backend::TensorRt || backend == Backend::Cuda) {
    OrtCUDAProviderOptions cuda_options{};
    cuda_options.device_id = 0;
    Ort::ThrowOnError(api.SessionOptionsAppendExecutionProvider_CUDA(options, &cuda_options));
  }

  return options;
}
}  // namespace

Detector::Detector(const DetectorConfig& config) : config_(config) {
  // 检查配置对不对
  if (config_.model_path.empty()) {
    throw std::invalid_argument("Detector: 模型路径不能为空");
  }
  if (config_.input_width <= 0 || config_.input_height <= 0) {
    throw std::invalid_argument("Detector: 输入尺寸必须大于 0");
  }
  if (config_.num_color <= 0 || config_.num_class <= 0) {
    throw std::invalid_argument("Detector: 颜色数和类别数必须大于 0");
  }
  if (config_.ignore_color < -1 || config_.ignore_color >= config_.num_color) {
    throw std::invalid_argument("Detector: ignore_color 超出颜色分支范围");
  }
  if (config_.backend != "auto" && config_.backend != "trt" && config_.backend != "cuda" && config_.backend != "cpu") {
    throw std::invalid_argument("Detector: 未知推理后端: " + config_.backend);
  }

  env_ = std::make_unique<Ort::Env>(config_.detector_verbose ? ORT_LOGGING_LEVEL_VERBOSE : ORT_LOGGING_LEVEL_WARNING,
                                    "auto_aim");

  if (!config_.engine_cache_dir.empty()) {
    std::error_code error;
    std::filesystem::create_directories(config_.engine_cache_dir, error);
    if (error) {
      std::clog << "[detector] 无法创建 engine 缓存目录 " << config_.engine_cache_dir << ": " << error.message()
                << "，本次不使用缓存\n";
      config_.engine_cache_dir.clear();
    }
  }

  std::string last_error;
  for (const Backend candidate : candidateBackends(config_.backend)) {
    if (candidate == Backend::TensorRt && !providerAvailable("TensorrtExecutionProvider")) {
      last_error = "ONNX Runtime 未提供 TensorRT EP";
      std::clog << "[detector] " << last_error << "，尝试下一个后端\n";
      continue;
    }
    if (candidate == Backend::Cuda && !providerAvailable("CUDAExecutionProvider")) {
      last_error = "ONNX Runtime 未提供 CUDA EP";
      std::clog << "[detector] " << last_error << "，尝试下一个后端\n";
      continue;
    }

    try {
      const auto session_start = std::chrono::steady_clock::now();
      Ort::SessionOptions options = makeOptions(candidate, config_);
      session_ = std::make_unique<Ort::Session>(*env_, config_.model_path.c_str(), options);
      const auto session_end = std::chrono::steady_clock::now();

      Ort::AllocatorWithDefaultOptions allocator;
      input_name_ = session_->GetInputNameAllocated(0, allocator).get();
      output_name_ = session_->GetOutputNameAllocated(0, allocator).get();
      input_names_ = {input_name_.c_str()};
      output_names_ = {output_name_.c_str()};

      const std::vector<std::int64_t> model_shape =
          session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      if (model_shape.size() != 4) {
        throw std::runtime_error("Detector: 模型输入不是 4 维张量");
      }

      // 动态维（-1）用配置值补，静态维必须和配置一致。
      input_shape_[0] = 1;
      input_shape_[1] = 3;
      input_shape_[2] = model_shape[2] > 0 ? model_shape[2] : config_.input_height;
      input_shape_[3] = model_shape[3] > 0 ? model_shape[3] : config_.input_width;
      if (input_shape_[2] != config_.input_height || input_shape_[3] != config_.input_width) {
        throw std::runtime_error("Detector: 模型输入尺寸 " + std::to_string(input_shape_[3]) + "x" +
                                 std::to_string(input_shape_[2]) + " 与配置 " + std::to_string(config_.input_width) +
                                 "x" + std::to_string(config_.input_height) + " 不一致");
      }

      // 首次 Run 可能触发 TensorRT engine 构建，必须在这里吃掉。
      std::vector<float> warmup_data(static_cast<std::size_t>(input_shape_[1] * input_shape_[2] * input_shape_[3]),
                                     0.0F);
      Ort::Value warmup_tensor =
          Ort::Value::CreateTensor<float>(memory_info_, warmup_data.data(), warmup_data.size(), input_shape_, 4);
      const auto warmup_start = std::chrono::steady_clock::now();
      session_->Run(Ort::RunOptions{nullptr}, input_names_.data(), &warmup_tensor, 1, output_names_.data(), 1);
      const auto warmup_end = std::chrono::steady_clock::now();

      backend_ = candidate;
      fp16_active_ = candidate == Backend::TensorRt && config_.fp16;

      const auto to_ms = [](const auto& begin, const auto& end) {
        return std::chrono::duration<double, std::milli>(end - begin).count();
      };
      std::clog << "[detector] backend=" << backendName(backend_) << " fp16=" << (fp16_active_ ? 1 : 0)
                << " cache=" << (config_.engine_cache_dir.empty() ? "off" : config_.engine_cache_dir)
                << " session=" << to_ms(session_start, session_end) << "ms warmup=" << to_ms(warmup_start, warmup_end)
                << "ms\n";
      return;
    } catch (const Ort::Exception& error) {
      last_error = error.what();
      session_.reset();
      std::clog << "[detector] " << backendName(candidate) << " 初始化失败: " << error.what() << "，尝试下一个后端\n";
    }
  }

  throw std::runtime_error("Detector: 没有可用的推理后端；最后一次错误: " + last_error);
}

Detector::Blob Detector::makeBlob(const cv::Mat& bgr) const {
  Blob input;  // 没人觉得在这个函数了应该叫output吗

  // 宽、高方向各算一个缩放比例，取较小的那个，不用头脑风暴长宽比比较
  const float scale_x = static_cast<float>(config_.input_width) / bgr.cols;
  const float scale_y = static_cast<float>(config_.input_height) / bgr.rows;
  input.info.scale = std::min(scale_x, scale_y);

  const int resized_width =
      std::max(1, std::min(config_.input_width, static_cast<int>(std::lround(bgr.cols * input.info.scale))));
  const int resized_height =
      std::max(1, std::min(config_.input_height, static_cast<int>(std::lround(bgr.rows * input.info.scale))));

  input.info.pad_x = (config_.input_width - resized_width) / 2;
  input.info.pad_y = (config_.input_height - resized_height) / 2;

  // 仿射变换
  cv::Mat resized;
  // x' = s * x + tx
  // y' = s * y + ty

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

  cv::resize(bgr, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);

  // 使用与训练一致的灰边填充
  cv::Mat canvas;
  cv::copyMakeBorder(resized, canvas, input.info.pad_y, config_.input_height - resized_height - input.info.pad_y,
                     input.info.pad_x, config_.input_width - resized_width - input.info.pad_x, cv::BORDER_CONSTANT,
                     cv::Scalar(124, 124, 124));

  // OpenCV 的 BGR 转成模型训练时使用的 RGB
  // 输出布局为 NCHW、CV_32F，并在这里完成 1/255 归一化
  cv::dnn::blobFromImage(canvas, input.blob, 1.0 / 255.0, cv::Size(config_.input_width, config_.input_height),
                         cv::Scalar(), true, false, CV_32F);

  return input;
}

ArmorDetections Detector::detect(const cv::Mat& bgr) {
  ArmorDetections detections;
  if (bgr.empty()) return detections;

  if (bgr.type() != CV_8UC3) {
    throw std::invalid_argument("Detector::detect 需要 CV_8UC3 的 BGR 图像");
  }

  // 三段计时：预处理、推理、后处理。加起来就是单帧检测的净开销。
  const auto stage_start = std::chrono::steady_clock::now();
  Blob input = makeBlob(bgr);
  if (input.blob.empty() || input.info.scale <= 0.0f) return detections;
  const auto preprocess_done = std::chrono::steady_clock::now();

  std::vector<Ort::Value> outputs;
  try {
    Ort::Value input_tensor =
        Ort::Value::CreateTensor<float>(memory_info_, input.blob.ptr<float>(), input.blob.total(), input_shape_, 4);
    outputs = session_->Run(Ort::RunOptions{nullptr}, input_names_.data(), &input_tensor, 1, output_names_.data(), 1);
  } catch (const Ort::Exception& error) {
    throw std::runtime_error(std::string("Detector: YOLO 前向推理失败: ") + error.what());
  }
  const auto inference_done = std::chrono::steady_clock::now();

  if (outputs.empty()) {
    throw std::runtime_error("Detector: 推理没有返回输出");
  }

  // 输出 [1,21,6300] 是连续的 float，包成无拷贝的 cv::Mat 头交给 decode。
  // outputs 必须活到 decode 结束；这里保持在函数作用域内即可。
  Ort::Value& output_tensor = outputs.front();
  const std::vector<std::int64_t> output_shape = output_tensor.GetTensorTypeAndShapeInfo().GetShape();
  if (output_shape.size() != 3) {
    throw std::runtime_error("Detector: 模型输出不是 3 维张量");
  }
  const int dims[3] = {static_cast<int>(output_shape[0]), static_cast<int>(output_shape[1]),
                       static_cast<int>(output_shape[2])};
  cv::Mat output(3, dims, CV_32F, output_tensor.GetTensorMutableData<float>());

  decode(output, input, bgr.cols, bgr.rows, detections);
  nms(detections, config_.nms_threshold);
  const auto postprocess_done = std::chrono::steady_clock::now();

  const auto to_ms = [](const auto& a, const auto& b) {
    return std::chrono::duration<double, std::milli>(b - a).count();
  };
  timings_.preprocess_ms = to_ms(stage_start, preprocess_done);
  timings_.inference_ms = to_ms(preprocess_done, inference_done);
  timings_.postprocess_ms = to_ms(inference_done, postprocess_done);

  return detections;
}

void Detector::decode(const cv::Mat& output, const Blob& input, int image_width, int image_height,
                      ArmorDetections& detections) const {
  const int expected_channels = config_.num_color + config_.num_class + kKeypointValues;

  if (output.dims != 3 || output.size[0] != 1 || output.size[1] != expected_channels) {
    const int actual_channels = output.dims == 3 ? output.size[1] : -1;
    throw std::runtime_error("Detector: 模型输出形状错误，期望通道数 " + std::to_string(expected_channels) + "，实际 " +
                             std::to_string(actual_channels));
  }

  const int candidate_count = output.size[2];
  const int class_offset = config_.num_color;
  const int keypoint_offset = config_.num_color + config_.num_class;

  // feature(f, c) = data[c + f * candidate_count]
  // 用feature()指针读取
  /*const cv::Mat flat = output.reshape(1, 1);
  const float* data = flat.ptr<float>(0);*/
  const float* data = output.ptr<float>(0);
  const auto feature = [&](int channel, int candidate) { return data[candidate + channel * candidate_count]; };

  ArmorDetections candidates;
  candidates.reserve(static_cast<std::size_t>(candidate_count * 0.1));

  for (int candidate{}; candidate < candidate_count; ++candidate) {
    // 类别分支取最大值，当作当前候选的类别和置信度
    int number{};
    float best_score = feature(class_offset, candidate);
    // 打擂台
    for (int cls = 1; cls < config_.num_class; ++cls) {
      const float score = feature(class_offset + cls, candidate);
      if (score > best_score) {
        best_score = score;
        number = cls;
      }
    }
    // 剔除置信不够的
    if (best_score < config_.score_threshold) continue;

    // 从颜色行中取最大值
    int color{};
    float best_color_score = feature(0, candidate);
    for (int current_color = 1; current_color < config_.num_color; ++current_color) {
      const float score = feature(current_color, candidate);
      if (score > best_color_score) {
        best_color_score = score;
        color = current_color;
      }
    }
    // 去除紫色和标明去除的颜色
    if (config_.ignore_purple && color == 3) continue;
    if (config_.ignore_color >= 0 && color == config_.ignore_color) continue;

    // 四个关键点从 letterbox 坐标还原到原图坐标
    ArmorDetection detection;
    bool valid = true;
    for (int keypoint{}; keypoint < kKeypointCount; ++keypoint) {
      const float x = feature(keypoint_offset + 2 * keypoint, candidate);
      const float y = feature(keypoint_offset + 2 * keypoint + 1, candidate);
      if (!std::isfinite(x) || !std::isfinite(y)) {
        valid = false;
        break;
      }

      detection.corners[keypoint].x = (x - static_cast<float>(input.info.pad_x)) / input.info.scale;
      detection.corners[keypoint].y = (y - static_cast<float>(input.info.pad_y)) / input.info.scale;

      // 任意关键点越界，都说明这个候选不完整，直接丢弃
      if (detection.corners[keypoint].x < 0.0f || detection.corners[keypoint].y < 0.0f ||
          detection.corners[keypoint].x >= static_cast<float>(image_width) ||
          detection.corners[keypoint].y >= static_cast<float>(image_height)) {
        valid = false;
        break;
      }
    }
    if (!valid) continue;

    // 生成斜矩形框，用来nms和center,height,width
    // 不要用它的点。由于angle不定，点的顺序无法保证
    detection.box = cv::minAreaRect(detection.corners);  // 成员改成 cv::RotatedRect rrect;

    if (detection.box.size.height <= 0.0f || detection.box.size.width <= 0.0f) continue;

    // 包装
    detection.score = best_score;
    detection.color = color;
    detection.number = number;
    detection.big = (number == kHeroClass || number == kBaseBottomClass);
    candidates.push_back(detection);
  }

  detections = std::move(candidates);  // 整体替换，不用写清除
}

void Detector::nms(ArmorDetections& detections, float iou_threshold) const {
  if (detections.empty()) return;

  // 先按分数从高到低排序，NMS 只保留高分框
  std::sort(detections.begin(), detections.end(),
            [](const ArmorDetection& left, const ArmorDetection& right) { return left.score > right.score; });

  std::vector<cv::RotatedRect> boxes;
  std::vector<float> scores;
  boxes.reserve(detections.size());
  scores.reserve(detections.size());

  for (const ArmorDetection& detection : detections) {
    boxes.push_back(detection.box);
    scores.push_back(detection.score);
  }

  std::vector<int> kept_indices;  // 此乃保留框的下标

  // 此处可以距离初筛优化
  // TODO:
  cv::dnn::NMSBoxes(boxes, scores, config_.score_threshold, iou_threshold, kept_indices);

  ArmorDetections kept;
  kept.reserve(kept_indices.size());
  for (int index : kept_indices) {
    kept.push_back(std::move(detections[index]));
  }
  detections = std::move(kept);
}
}  // namespace detector
