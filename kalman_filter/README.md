# auto_aim / kalman_filter

基于 YOLO、PnP 和 9 维 EKF 的装甲板状态估计程序。

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
├── estimation/   9 维 EKF
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

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_TESTING=ON

cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

MVS 不在默认位置时：

```bash
cmake -S . -B build -DMVS_ROOT=/path/to/MVS
```

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
| `--help` | 查看完整帮助 |

输出：

| 文件 | 内容 |
|---|---|
| `output/output_video.avi` | 检测框、角点和 EKF 状态叠加 |
| `output/data.csv` | 时间、9 维状态和本帧重投影误差 |

CSV 的历史格式保留下来，因此数据行末尾可能追加数量不固定的重投影误差列。

## 实时模式

```bash
./build/kalman_filter --live \
  --camera-index 0 \
  --camera-width 1440 \
  --camera-height 1080 \
  --serial /dev/ttyUSB0 \
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

时间戳来源：

1. `open()` 后 `MV_CC_GetImageBuffer` 之前，相机已经 `StartGrabbing`，此时读 `DeviceTimestamp` 节点，与 `steady_clock` 配对采样 24 次，取中位数得到钟差。
2. 每帧把 `nDevTimeStampHigh/Low` 换算到主机单调钟，作为 `frame.timestamp`。
3. 校准失败、节点不支持或读数为 0 时自动退回收帧时刻，不影响取流。
4. 设备戳落在曝光区间的哪个位置需要台架实测：改曝光时间看「收帧时刻 − 映射时刻」是否变化，然后设置 `--camera-exposure-anchor`。
5. 绝对偏差用 `--camera-time-offset-us` 吸收，正值表示把帧时间往后挪。

按 `Ctrl+C` 安全停止。

当前实时模式只接收数据并估计状态，不会自动生成或发送云台控制指令。`RealtimePipeline::send()` 已预留，后续应在弹道、延迟和安全门限完成后接入。

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
- 实时相机的分辨率、曝光、增益和焦距必须与内参标定时一致。
- C 板反馈中的 `DWT_stamp` 当前按 32 位微秒、可回绕处理。
- 相机杆臂和真实安装外参尚未建模；当前假设 C 板给出的 `q` 已经完成相机姿态换算。
- `total_delay = process_latency + comm_latency + fly_time` 的统计框架已具备，但通信延迟和飞行时间仍需现场测量或由弹道模块提供。
- 实时模式不会自动发送控制指令，避免在时间同步和弹道未验证时产生危险动作。

## 测试

CTest 覆盖：

- 64 字节协议打包、解包和校验
- 伪终端半包、垃圾字节和串口恢复
- DWT 时间回绕与时钟映射
- 姿态历史插值、历史边界和短时预测
- 处理、通信、飞行时间及总延迟统计

```bash
ctest --test-dir build --output-on-failure
```
