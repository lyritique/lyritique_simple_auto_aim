# kalman_filter 视觉第三周作业

> 一段视频，一份 IMU 四元数，九个状态量。
。

## 组织架构

从「YOLO 检测到的装甲板 4 个角点」到「整车 9 维状态」。

```
视频帧 ──┬─→ YOLO(onnx) 检测 4 角点 ──→ cornerSubPix 精修（择优）
         │                                  │
         │                                  ↓
         │                        IPPE 解算 4 点 PnP（两解消歧）
         │                                  │  (x, y, z, yaw)_armor
         │                                  ↓
         │                    球坐标 (azi, ele, dist) → 观测噪声 R
         │                                  │
IMU 四元数 ─→ slerp 对齐(offset 9ms) ──────→ │   ← 目前只做了对齐，还没进滤波
                                            ↓
                              9 维 EKF：predict → 匹配 → update
                                            │
              ┌─────────────────────────────┼────────────────────────┐
              ↓                             ↓                        ↓
   output/output_video.avi         output/data.csv         右上角实时状态量
   （检测框 / 角点 / 状态）      （9 维状态时间序列）      （pos / vel / yaw / vyaw / r）
```

状态量：`X = [px, py, pz, yaw, vx, vy, vz, vyaw, r]`
（整车中心位置、整车朝向、速度、角速度、装甲板旋转半径）

## 目录结构

| 路径 | 职责 | 行数 |
| --- | --- | ---: |
| `include/armor_detector.hpp` / `src/armor_detector.cc` | YOLOv8-onnx 推理 + letterbox 反变换 + 解码 + NMS | 103 / 311 |
| `include/data_interaction.hpp` / `src/data_interaction.cc` | 装甲板物点模板、内参、IPPE 解算、两解消歧、重投影误差 | 110 / 201 |
| `include/kalman_filter_task.hpp` / `src/kalman_filter.cc` | 9 维 EKF：运动模型、观测模型、id 匹配、更新 | 105 / 389 |
| `include/imu_alignment.hpp` / `src/imu_alignment.cc` | IMU 读表、lower_bound 定位、四元数 slerp | 39 / 101 |
| `include/draw.hpp` / `src/draw.cc` | 画检测框、画状态面板 | 48 / 193 |
| `src/main.cc` | 主循环：时间戳、检测、PnP、喂 EKF、写视频/CSV | 398 |
| `model/Infantry_fp32.onnx` | 检测模型（9 类机器人数 × 4 色） | — |
| `material/` | 视频 + IMU 内录 + 内参 + 作业说明 | — |
| `tools/pnp_sanity.py` | 纯 numpy 的 PnP 自检（角点顺序 / 两解 / 噪声灵敏度） | — |
| `tools/pnp_log_analyze.py` | 解析 `easySolver()` 的 `sol/rms/n·t/yaw` 日志，算选解翻转率 | — |
| `tools/avi_probe.py` | 从输出视频抽帧，方便对着帧看检测和状态 | — |
| `tools/plot_states.py` | 读 CSV 画 9 条状态曲线（WSL，需要 numpy + matplotlib） | — |

## 依赖（WSL / Ubuntu）

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev libeigen3-dev
# 画图用（可选）
sudo apt install -y python3-pip && pip3 install numpy pandas matplotlib
```

- OpenCV ≥ 4.5（用到了 `SOLVEPNP_IPPE`、`solvePnPGeneric`、`dnn`）
- Eigen ≥ 3.4
- GCC ≥ 11（`CMakeLists.txt` 里写了 C++23；**如果报 `unknown argument -std=gnu++23`，把
  `CMAKE_CXX_STANDARD` 改成 20 即可** —— 代码里用到的 `std::string::ends_with` 是 C++20）

## 构建 & 运行

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
./build/kalman_filter
```

不带参数时用默认路径（`material/…avi`、`material/…txt`、`output/`），**必须在仓库根目录执行**，
因为模型路径 `model/Infantry_fp32.onnx` 是相对路径。

想自己指定路径，注意 `main.cc` 的参数解析有几处边界条件不太灵光，
目前唯一稳妥的写法是**四个参数**：

```bash
./build/kalman_filter "material/2026-05-20_23-57-31 (1).avi" \
                       "material/2026-05-20_23-57-31.txt" \
                       "output" \
                       "dummy"      # 第 4 个参数只是为了让 argc>4 成立
```

- 想用 VS Code 的 clangd：`.vscode/settings.json` 里写死了
  `/home/lyritique/project/auto_aim/kalman_filter/build`，换成你自己的路径，
  或者直接用 `--compile-commands-dir=build`。

## 输出

| 文件 | 内容 |
| --- | --- |
| `output/output_video.avi` | MJPEG，1440×1080，30 fps。右上角是 9 维状态；黄点是 PnP 用的角点，紫点是精修后的角点 |
| `output/data.csv` | 每帧一行：`time,valid,px,py,pz,yaw,vx,vy,vz,vyaw,r`，**后面还跟着本帧的若干个重投影误差** |



## 踩过的坑（历史记录）

> 下面这份清单是对开发过程中某一版代码的记录，**部分问题在 `repairEKF` 分支里已经修掉**；
> 交付版本的实际指标看报告 `doc/视觉第三周作业_吴作庆.md` 的 1.4 节。

1. **两解消歧实际上是「按 rms 抛硬币」** —— `data_interaction.cc` 里
   `if (R.col(0).dot(tvecs) >= 0) continue;` 只丢 `n·t > 0` 的解；而实测日志里
   **所有解都满足 `n·t < 0`**，所以这条几何判据一个解也没拦住，两个 IPPE 解都进门。
   接着 `cost = rms + kyaw·Δyaw²` 里 `kyaw = 0`，于是「选谁」全凭那 0.05~1.2 px 的残差差，
   而两解的 yaw 差 60~220° —— 选解在相邻帧之间来回翻，这就是 yaw 台阶跳变的直接原因。
   证据与修法见 [docs/PNP排查记录.md](docs/PNP排查记录.md)。
2. **`previous_yaw` 从来没被更新** —— `pnpSolver::input()` 只清空输出，不记上次的 yaw。
   想让上面的 `kyaw` 起作用，必须先补上 `previous_yaw = best_yaw;`，
   否则 cost 会变成「惩罚 yaw 偏离 0」，越用越歪。
3. **CSV 列数不齐** —— 表头 11 列，数据行后面又追加了 0~2 个重投影误差，
   还有 `csv << image_time << "偏离过大"` 这种**不带逗号**的行。
   `pandas.read_csv` 会直接报错或者读出一堆 NaN —— 这大概就是你看到「CSV 没有数据」的原因。
   `tools/plot_states.py` 用了容错解析，先顶着用。
4. **IMU 只对齐、没进滤波** —— `alignImuToImage()` 的返回值 `cam_imu` 算出来就扔了。
   `yaw_odom` / `yaw_kf` 两条曲线目前是空的。
5. **`init()` 假设「第一眼看到的就是 0 号板」** —— `measurement.testResult.id = 0` 写死。
   四块板 yaw 差 90°，押错了整车 yaw 就整体偏 90°。而 `id_list` 的 0 号是从
   `storedID{}` 默认值来的，不是真的匹配出来的。
6. **`P *= 0.1`** 把位置方差和 yaw 方差都设成 0.1，量纲都不管了；`r` 的方差被写成
   `kRadiusSigma² = 1e-4`，`p_r = 1e-5`，于是 `r` 卡死在 0.26 附近（CSV 里 `r` 全程
   0.248~0.260），等于把「装甲板半径」当已知量，白丢一个自由度。
7. **`reset()` 会把 `F` 清成零矩阵**，而 `F` 只在构造函数里 `setIdentity()` 过一次。
   注释里写「多次偏离应该 reset」——**一旦真调用 reset，predict 立刻失效**。
8. **命令行参数解析**：`argc > 4` 才读 `args[3]` 当输出目录，而 `args[2]` 才是输出目录。


