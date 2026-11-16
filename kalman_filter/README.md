# auto_aim / kalman_filter

基于 YOLO、PnP 和 11 维 EKF 的装甲板状态估计程序。

支持两种运行方式：

- `--replay`：读取离线 AVI + IMU 文件，生成带检测和状态叠加的视频与 CSV。
- `--live`：读取 MVS 工业相机和 C 板云台反馈，实时估计目标状态。

## 数据流

```text
CameraSource
  → TimestampedFrame
  → 按帧时间获取云台姿态
  → FrameProcessor
       ├─ YOLO 检测
       ├─ 角点精修
       ├─ PnP
       └─ EKF predict / update
  → ReplayPipeline：视频 + CSV
  → RealtimePipeline：实时状态
```

检测、PnP 和 EKF 都放在共享的 `FrameProcessor` 中，实时与回放不会各维护一套算法。

## 目录

```text
src/
├── app/          命令行、模式分发
├── core/         统一配置、带时间戳帧
├── vision/       检测、装甲板几何、PnP
├── estimation/   11 维 EKF
├── runtime/      共享处理器、实时/回放 Pipeline
├── sensors/      相机接口、MVS 相机、AVI 视频源
├── timing/       时钟同步、姿态历史、延迟统计、离线 IMU
├── transport/    64 字节协议、Linux 串口
└── debug/        检测框和状态面板绘图
```

主要配置默认值集中在 `src/core/config.hpp`。检测模型和素材路径相对于运行时工作目录。

## 构建

依赖：

- CMake ≥ 3.16
- OpenCV ≥ 4.5
- Eigen ≥ 3.4
- GCC 支持 C++23
- Hikrobot MVS SDK，默认安装在 `/opt/MVS`
- ONNX Runtime GPU 包 ≥ 1.22（含 CPU / CUDA / TensorRT EP）
- TensorRT 10.9 + CUDA 12.8 + cuDNN 9（TensorRT 版本必须与 ORT 预编译包对齐）

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DORT_ROOT=/path/to/onnxruntime \
  -DBUILD_TESTING=ON

cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

MVS / ORT 不在默认位置时：

```bash
cmake -S . -B build \
  -DMVS_ROOT=/path/to/MVS \
  -DORT_ROOT=/path/to/onnxruntime
```

运行时需要让动态加载器找到 ONNX Runtime 的 provider 以及 TensorRT / CUDA 库：

```bash
export LD_LIBRARY_PATH=/path/to/onnxruntime/lib:/path/to/TensorRT/lib:/path/to/cuda/lib64:$LD_LIBRARY_PATH
```

## 检测后端

推理通过 ONNX Runtime 执行，默认按 **TensorRT → CUDA → CPU** 自动降级；启动时会打印实际生效的后端。检测参数：

| 参数 | 含义 |
|---|---|
| `--detector-backend auto\|trt\|cuda\|cpu` | 选择后端，默认 `auto` |
| `--no-fp16` | 关闭 TensorRT FP16，做精度对照用 |
| `--trt-cache DIR` | TensorRT engine / timing 缓存目录，默认 `model/trt_cache`，传空字符串禁用 |
| `--detector-verbose` | 打印 ORT / TensorRT 初始化日志 |

TensorRT 首次运行需要构建 engine，会在 `model/trt_cache` 下生成缓存；冷启动通常 1~3 分钟（取决于 builder 优化级别和显存/内存），之后复用缓存仍需数秒到十几秒完成反序列化。**缓存与 GPU 型号、模型、ORT / TensorRT 版本、FP16 设置绑定，升级任一组件或换机器后要删掉重建；不要提交到仓库。** 部署机建议先跑一次 `--replay` 预热缓存，再进实时模式。

现场若 TensorRT 版本不匹配，直接 `--detector-backend cuda` 或 `cpu`，不需要改代码。

> 注意：当前模型在 CUDA EP 下会被插入较多 Memcpy 节点，实测单帧推理反而比 CPU 慢；TRT 不可用时优先试 `cpu`，`cuda` 只作为排查手段。

## 离线回放

```bash
./build/kalman_filter --replay \
  --video "material/2026-05-20_23-57-31 (1).avi" \
  --imu "material/2026-05-20_23-57-31.txt" \
  --output output
```

可选参数：

| 参数 | 含义 |
|---|---|
| `--imu-offset-ms MS` | IMU 相对图像的时间偏移，离线默认 `9 ms` |
| `--max-distance M` | 送入 EKF 的最大目标距离，默认 `2.5 m` |
| `--calibration PATH` | 相机标定文件（OpenCV YAML），覆盖内置参考内参 |
| `--help` | 查看完整帮助 |

输出：

| 文件 | 内容 |
|---|---|
| `output/output_video.avi` | 检测框、角点和 EKF 状态叠加 |
| `output/data.csv` | 时间、11 维状态和本帧重投影误差 |

CSV 的历史格式保留下来，因此数据行末尾可能追加数量不固定的重投影误差列。

## 实时模式

```bash
./build/kalman_filter --live \
  --camera-index 0 \
  --camera-width 1440 \
  --camera-height 1080 \
  --serial /dev/ttyACM0 \
  --baud 115200
```

常用参数：

| 参数 | 含义 |
|---|---|
| `--camera-fps F` | 固定相机帧率，`0` 表示不修改 |
| `--exposure-us US` | 固定曝光时间，`0` 表示不修改 |
| `--gain G` | 固定增益，负数表示不修改 |
| `--camera-time-source S` | `device` 用相机硬件戳，`host` 用收帧时刻；默认 `device` |
| `--camera-time-offset-us US` | 时间戳常数偏移，台架标定后写入 |
| `--camera-exposure-anchor R` | 设备戳在曝光区间的比例：`0` 开始、`0.5` 中点、`1` 结束 |
| `--no-chunk-exposure` | 关闭 Chunk-Exposure，不读每帧曝光 |
| `--camera-timing-log` | 打印时钟校准结果和每帧时间戳 |
| `--comm-latency S` | 串口到云台执行延迟估计 |
| `--max-distance M` | 送入 EKF 的最大目标距离 |
| `--calibration PATH` | 相机标定文件（OpenCV YAML），覆盖内置参考内参 |

时间戳来源：

1. `open()` 后 `MV_CC_GetImageBuffer` 之前，相机已经 `StartGrabbing`，此时读 `DeviceTimestamp` 节点，与 `steady_clock` 配对采样 24 次，取中位数得到钟差。
2. 每帧把 `nDevTimeStampHigh/Low` 换算到主机单调钟，作为 `frame.timestamp`。
3. 校准失败、节点不支持或读数为 0 时自动退回收帧时刻，不影响取流。
4. 设备戳落在曝光区间的哪个位置需要台架实测：改曝光时间看「收帧时刻 − 映射时刻」是否变化，然后设置 `--camera-exposure-anchor`。
5. 绝对偏差用 `--camera-time-offset-us` 吸收，正值表示把帧时间往后挪。

按 `Ctrl+C` 安全停止。

### 云台指令

默认只算不发。`--aim-debug` 每 0.5 s 打一行，把解出来的指令和 C 板自报的姿态并排打出来；
确认过坐标系再用 `--send-control` 真正写串口。

| 参数 | 含义 |
|---|---|
| `--aim-debug` | 打印解算指令 + C 板自报 yaw/pitch，判断坐标系约定 |
| `--send-control` | 真正把 `VisionToGimbal` 写进串口 |
| `--bullet-speed F` | C 板没报弹速时的兜底值，m/s |
| `--command-mode N` | 指令里的 `mode` 字节，含义由 C 板定义，默认 0 |
| `--up-axis y\|z` | C 板世界系哪根轴朝天，默认 `z`（`yaw=atan2(y,x)`、`pitch=asin(z)`） |
| `--yaw-sign F` | yaw 正方向，反了传 `-1` |
| `--pitch-sign F` | pitch 正方向，`1` 为抬头为正，`-1` 取反 |
| `--no-cam-remap` | 关闭相机系 → C 板体轴换轴，排查坐标系时用 |

解算流程（`src/control/aim_solver.cpp`）：

1. 按 `处理延迟 + 通信延迟 + 飞行时间` 把 EKF 状态外推到命中时刻，飞行时间和距离互相耦合，迭代两次；
2. 在预测时刻挑出正对相机的那块装甲板，取板心作为瞄准点（半径按奇偶 id 叠加 Δr、Δz）；
3. 换轴并转回 C 板世界系。

第 3 步是重点。C 板的四元数描述的是它的 IMU 体轴（x 前、y 左、z 上），实测它的 `yaw`/`pitch`
正是体轴 +x 的方位角/仰角（两种姿态都对到 0.01°）；而 EKF 的观测向量在相机系里（x 右、y 下、z 前）。
两者差一个 90°，所以出口要按 `q_now · M · p_cam` 走：

```text
p_cam = q_now⁻¹ · q_init · p_C0      // C0 退回当前相机系
d     = q_now · M · p_cam            // 换轴后转到世界系，M 见 aim_solver.cc
yaw   = atan2(d.y, d.x)
pitch = asin(d.z)
```

C0 是"首帧相机系"，相机耷拉着时它也跟着歪，而 `q_init`/`q_now` 的复合正好把这个耷拉角抵消掉。
所以发出去的是**绝对世界角**：开机时相机耷拉 20°，pitch 就会是 -20° 附近，而不是 0。
自检办法：目标在画面正中时，`cmd yaw/pitch` 应该和 `board yaw/pitch` 基本相等。

首次实测建议：

```bash
./build/kalman_filter --live --aim-debug ...          # 只看不发
```

对照 `[aim]` 那一行：`board pitch` 应该和"y 朝天"或"z 朝天"其中一组建模对得上。
如果 `board pitch` 和 `asin(z)` 一致而和 `asin(y)` 对不上，就加 `--up-axis z`；
如果数值一致但符号相反，就加 `--pitch-sign -1`。对好之后再加 `--send-control`。

离线验证可以用回放：

```bash
./build/kalman_filter --replay --video ... --imu ... --aim-debug
```

回放会每 0.5 s 打一行 `[aim]`，用来看角度跟不跟得上；回放不发任何数据。

弹道下坠补偿（`compensate_drop`）目前没有接进 CLI，默认关闭，等弹道模型标定后再开。

## 坐标系与时间

这是最容易出错的部分。

```text
图像系：左上角原点，x 右、y 下
相机系 C：x 右、y 下、z 前
C 板反馈 q：相机系 → C 板参考世界系
算法世界 C0：初始化帧的相机坐标系
```

相对旋转：

```text
R_C0C = q_init⁻¹ · q(t)
p_C0 = R_C0C · p_camera
```

装甲板 yaw：

```text
yaw = atan2(n_C0.x, n_C0.z)
```

当前 EKF 约定：

```text
水平面：x-z
竖直轴：y
正 yaw：+z 转向 +x
```

实时模式使用 `ClockSync + GimbalFeedbackHistory`：

1. 把 C 板 DWT 时间映射到上位机单调时钟。
2. 用相机帧时间查询前后两个反馈样本。
3. 在历史区间内做四元数 SLERP 和标量插值。
4. 图像对齐不使用未来外推；未来外推只允许用于控制预测，并有最大时长限制。

## 当前限制

- MVS 封装默认把相机 `DeviceTimestamp` 映射到主机单调钟作为帧时间；校准失败时退回收帧时刻。设备戳对应曝光开始/中点/结束的哪一种，仍需台架实测后通过 `--camera-exposure-anchor` 指定。
- 实时相机的分辨率、曝光、增益和焦距必须与内参标定时一致。内参默认是材料里的参考值，可用 `--calibration` 指向 `camera_calibration` 输出的 YAML；画面尺寸与标定不一致时会在首帧打印一次警告。
- C 板反馈中的 `DWT_stamp` 当前按 32 位微秒、可回绕处理。
- 相机杆臂和真实安装外参尚未建模；当前假设 C 板给出的 `q` 已经完成相机姿态换算。
- `total_delay = process_latency + comm_latency + fly_time` 的统计框架已具备，但通信延迟和飞行时间仍需现场测量或由弹道模块提供。
- 控制指令默认只算不发；`--send-control` 打开前请先用 `--aim-debug` 核对坐标系约定和角度符号。
- C 板世界系的竖轴方向（`--up-axis`）和 pitch 正方向（`--pitch-sign`）尚未实测确认，默认按"y 朝天、抬头为正"。
- 指令里的 `time_stamp` 是该指令的生成时刻，换算到 C 板时钟、单位毫秒（与 `DWT_stamp` 一致）。C 板若按其它单位解析需要再对一次。
- 相机杆臂（相机光心与云台旋转中心的偏移）尚未建模，近距离大角度时会有误差。

## 测试

CTest 覆盖：

- 64 字节协议打包、解包和校验
- 伪终端半包、垃圾字节和串口恢复
- DWT 时间回绕与时钟映射
- 姿态历史插值、历史边界和短时预测
- 处理、通信、飞行时间及总延迟统计
- 11 维 EKF 的两对装甲板结构差异（Δr/Δz）收敛
- 瞄准解算的耷拉角补偿、提前量、pitch 符号与竖轴约定

```bash
ctest --test-dir build --output-on-failure
```
