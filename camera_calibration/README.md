# camera_calibration（相机标定程序）

依托 `kalman_filter_fixed/` 新建的**独立工程**：同样的 CMake + C++23 + OpenCV 骨架、同样的
`include/ src/ tools/` 分层和中文注释风格，专门用来做**棋盘格离线相机标定**，产出的内参可以
直接回填 fixed 版的 `pnpSolver`。**fixed 版里的任何文件都没有被改动。**

为什么要单独做这个：fixed 版 `include/data_interaction.hpp` 里的 `pnpConfig::camParams`
把相机内参硬编码了（来源是 `material/说明.txt`，作业直接给的材料）。真实项目里换镜头、
换分辨率、重调焦距都要重新标定；这一步独立出来，标完把结果塞回去，PnP 和 EKF 的代码
一行都不用改。

| | kalman_filter_fixed | camera_calibration |
| --- | --- | --- |
| 构建 | CMake + C++23 + Eigen + OpenCV | CMake + C++23 + OpenCV（不碰 Eigen / ONNX / 视频 / IMU） |
| 内参 | `camParams` 硬编码 | `config/reference_intrinsics.yml` 作对照，计算得出新内参 |
| 单位 | 装甲板模板 mm、`tvec` 最后除 1000 转 m | 棋盘方格 mm（只影响外参，不影响内参） |
| 输出 | `output_video.avi` / `data.csv` | `calibration.yml` / `camParams_snippet.txt` / 误差图 |
| 数学 | IEKF 估计整车状态 | 张正友平面标定 + LM 批量优化内参 |

> 说明：这是**离线标定**，不是「用 EKF 在线估计内参」。内参是常量，标准做法是拿多张已知
> 姿态的标定板做批量最小二乘；fixed 版 EKF 的职责是估计装甲板/整车状态，内参是它的
> **已知输入**。这个程序就是负责把那个输入算准。想做在线自标定得另起一个题目（把 K/D
> 放进状态量做滑窗优化），本工程留了 YAML 接口，扩展起来很方便。

---

## 一、快速开始（WSL Ubuntu 22.04 / OpenCV 4.5.4 / g++ 11，与 fixed 版同环境）

```bash
cd camera_calibration
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j16

./build/camera_calibration --help          # 看全部参数
./build/camera_calibration                 # 没准备图片时自动跑「零素材自检」
```

**零素材自检** 是最该先跑的一条：程序拿 fixed 版给的内参当真值，用
`cv::projectPoints` 生成 20 个不同姿态的虚拟棋盘角点，再自己标定回来。
如果整条链路（三维点顺序、投影模型、畸变系数顺序、标定 flags）没问题，结果应该
几乎精确等于真值：

- 整体 RMS 应在 **0.01 px 以下**（理想观测）；
- 对照表里 `fx/fy/cx/cy/k1/k2/p1/p2/k3` 的差值应接近于 0
  （焦距相对误差 < 0.1%，即 `fx` 偏差在几个像素以内）；
- 「换用本次内参后的投影位移」应 **< 0.1 px**。

差得离谱就说明代码或参数被改坏了，比拿真照片调半天有效得多。想看看噪声的影响：
`./build/camera_calibration --synthetic --synthetic-noise 0.3`。

---

## 二、真实标定流程

### 1. 做一张标定板

```bash
python3 tools/make_chessboard.py --cols 12 --rows 9 --square 25 --out chessboard.svg
```

- `--cols/--rows` 是**格子数**，比标定程序要填的**内角点数**各多 1：
  12×9 格 → 11×8 内角点 → 程序里写 `--cols 11 --rows 8`。**这里填错是最常见的失败原因。**
- 打印必须选「实际大小 / 100% 缩放」，贴到硬纸板上保持平整，四周白边别裁掉
  （`findChessboardCorners` 靠白边判断棋盘外轮廓）。
- **打印完用尺子实测** 10 个格子的总长再除以 10，把实测值传进 `--square`。
  打印机缩放差 2%，`fx/fy` 就会整体偏 2%——这比角点检测误差大得多。

### 2. 拍 12~20 张不同姿态的照片

```bash
mkdir images && cp 拍好的照片/* images/
./build/camera_calibration --images images --cols 11 --rows 8 --square 25 --preview
```

姿态要求（程序会在终端里报告覆盖情况）：
- 棋盘中心要分布在画面**四角和中心**，程序按 3×3 分区统计，建议覆盖 ≥ 6 格；
- 要有明显**倾斜**的板（程序会打印每视图倾角，别全挤在 0° 附近），
  否则 `fx/fy` 与板距离强耦合，内参看着收敛其实不可信；
- 距离可以变化，但每次都要**完整**出现在画面里、四个角不能贴边。

也可以直接从视频抽帧或实时采集：

```bash
./build/camera_calibration --video calib.avi --stride 5 --min-move 25
./build/camera_calibration --camera 0      # 空格抓帧、q/ESC 结束，需要图形界面
./build/camera_calibration --synthetic     # 无素材自检
```

视频和摄像头会自动跳过与上一张角点平均位移小于 `--min-move` 的重复画面。

### 3. 看结果、画误差图

```bash
python3 tools/plot_per_view_error.py output/per_view_error.csv output/error.svg
```

RMS 判读：`< 0.5 px` 优秀，`< 1.0 px` 可用，`> 2.0 px` 基本要重拍或排查。

---

## 三、目录结构

```
camera_calibration/
├── CMakeLists.txt                       # 与 fixed 版同款配置，只留 OpenCV
├── README.md
├── include/
│   ├── calib_config.hpp                 # BoardSpec / Options / 命令行解析声明
│   ├── board_detector.hpp               # View、棋盘检测、合成自检视图
│   ├── calibrator.hpp                   # CalibrationResult、Calibrator
│   └── calib_io.hpp                     # YAML 读写、对照、报告、代码片段
├── src/
│   ├── calib_config.cc                  # 参数解析 + 合法性检查 + 三维点生成
│   ├── board_detector.cc                # 角点检测/亚像素精修/去重/合成投影
│   ├── calibrator.cc                    # calibrateCamera、逐视图误差、离群剔除
│   ├── calib_io.cc                      # 落盘、参考内参、像素级对照
│   └── main.cc                          # CLI + 四种输入源调度
├── config/
│   └── reference_intrinsics.yml         # fixed 版给的内参（对照/自检真值）
├── tools/
│   ├── make_chessboard.py               # 纯标准库生成可打印棋盘格 SVG
│   └── plot_per_view_error.py           # 纯标准库画逐视图误差 SVG
└── output/                              # 运行后生成（已 gitignore）
```

---

## 四、参数一览

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `--images <dir>` | `images` | 读目录里的 jpg/png/bmp/tif |
| `--video <file>` | — | 从视频抽帧 |
| `--camera [id]` | `0` | 摄像头实时采集（空格抓帧、q 结束） |
| `--synthetic` | — | 用 fixed 版内参合成虚拟观测做自检 |
| `--cols / --rows` | `11 / 8` | **内角点**数，不是格子数 |
| `--square <mm>` | `25.0` | 方格边长，建议实测 |
| `--output <dir>` | `output` | 输出目录 |
| `--reference <yml>` | `config/reference_intrinsics.yml` | 对照用内参；读不到会用内置兜底值 |
| `--fix-k3` | 关 | 固定 `k3 = 0`（fixed 版给的文件里 k3 就是 0，自检时可打开对比） |
| `--fix-principal` | 关 | 主点固定在图像中心，数据少时防跑飞 |
| `--zero-tangent` | 关 | 强制 `p1 = p2 = 0` |
| `--preview` | 关 | 额外保存角点叠加图（最多 12 张） |
| `--no-refine` | 关 | 关掉「剔除离群视图后重标一次」 |
| `--no-compare` | 关 | 不打印与 fixed 版内参的对照 |
| `--min-views / --max-views` | `8 / 40` | 有效视图下限 / 上限 |
| `--min-move <px>` | `25` | 视频/摄像头去重阈值 |
| `--stride <n>` | `3` | 视频每隔几帧检测一次 |
| `--synthetic-views / --synthetic-noise` | `20 / 0` | 合成视图数量 / 角点高斯噪声 σ(px) |

不带任何参数且 `images/` 不存在时，程序自动进入 `--synthetic`，不会直接报错退出。

---

## 五、输出文件

| 文件 | 内容 |
| --- | --- |
| `output/calibration.yml` | `camera_matrix` / `distortion_coefficients` / 图像尺寸 / RMS / 每张图误差。字段名与 `config/reference_intrinsics.yml` 一致，可被其它 OpenCV 程序直接读 |
| `output/camParams_snippet.txt` | **可直接粘贴回 fixed 版**的 `camParams` 结构体，另附「改成从 YAML 读」的方案 |
| `output/per_view_error.csv` | 每张图：是否参与、第一轮/最终 RMS、板面倾角、棋盘中心像素位置 |
| `output/undistort_preview.jpg` | 去畸变前后并排对比，直线有没有被掰直一眼可见 |
| `output/corners_XX.jpg` | `--preview` 时的角点检测叠加图 |

---

## 六、标定原理（张正友平面标定）

1. **三维点**：标定板放在 `z = 0` 平面，第 `(c, r)` 个内角点的坐标是
   `(c·square, r·square, 0)`，顺序和 `findChessboardCorners` 的输出一一对应
   （`BoardSpec::objectPoints()`）。
2. **单应矩阵**：每个姿态下，板平面到像平面是一个 3×3 单应 `H`。由
   `h1 = λ·K·r1`、`h2 = λ·K·r2` 和旋转矩阵列正交（`r1⊥r2`、`|r1|=|r2|`），
   每个姿态给出两个关于 `KᵀK⁻¹` 的约束；≥ 3 个不同姿态即可闭式解出内参初值。
3. **畸变模型**（OpenCV 标准针孔模型，顺序 `k1, k2, p1, p2, k3`）：

   ```
   x' = x·(1 + k1 r² + k2 r⁴ + k3 r⁶) + 2 p1 x y + p2 (r² + 2x²)
   y' = y·(1 + k1 r² + k2 r⁴ + k3 r⁶) + p1 (r² + 2y²) + 2 p2 x y
   u = fx·x' + cx,   v = fy·y' + cy            (r² = x² + y²)
   ```

4. **联合优化**：把内参、畸变、每张图的 `rvec/tvec` 全部作为未知量，用 LM 最小化
   所有角点的重投影误差平方和（`cv::calibrateCamera`），返回的 `rms` 就是均方根。
5. **角点精度**：`findChessboardCorners` 只给整像素，先做 FAST_CHECK 快速筛选、
   漏检再退回完整检测；命中后用 `cornerSubPix` 在灰度梯度上精修到亚像素，
   否则 ±0.5 px 的量化误差会直接顶高 RMS。
6. **离群剔除**：按 `中位数 + 3 × 1.4826 × MAD` 判坏图，再重标一次。用中位数/MAD
   而不是均值/标准差，是因为一两张角点检错的图会把均值和 σ 一起拉爆，反而漏掉它自己；
   阈值另加了 `med+0.15` 和 `0.30 px` 两个下限，避免理想合成数据里 MAD≈0 误杀。
   同时保证至少保留一半视图，防止把好数据砍光。
7. **像素级对照**：把画面上 11×11 个像素点用 fixed 版内参 `undistortPoints` 反投影成
   无畸变射线，再用本次内参 `projectPoints` 投回像素，统计位移的 RMS/最大值。
   这比单看 `fx` 差了多少更能反映「换了内参，PnP 结果会偏多少像素」。

---

## 七、怎么回填 fixed 版（重点）

`output/camParams_snippet.txt` 里已经生成好完整结构体，两种方案：

**方案 A（改数值，最快）**：用它替换
`kalman_filter_fixed/include/data_interaction.hpp` 里的 `pnpConfig::camParams` 成员初值。
`pnpSolver::pnpSolver()` 会照旧拼出 `camera_matrix_` 和 `distortion_coeffs_`，
PnP / EKF / 画板代码**一行都不用改**。

**方案 B（改成读 YAML，长期更舒服）**：把 `output/calibration.yml` 拷到 fixed 版根目录，
按 snippet 里的示例把 `pnpSolver()` 构造函数体换成 `cv::FileStorage` 读取，
以后重标只要换文件、不用重新编译。

对接时三个容易踩的点：

- 畸变顺序完全一致：`(k1, k2, p1, p2, k3)`，**不需要重排**；
- 图像分辨率要一致：fixed 版那段视频是 `1440×1080`，用别的分辨率标出来的 `cx/cy`
  不能直接用（要按比例缩放内参）；
- 单位：棋盘 `--square` 用 mm 只影响外参，fixed 版装甲板模板也是 mm，
  `easySolver()` 里 `/1000` 转 m 的逻辑不受影响。

---

## 八、常见问题

| 现象 | 原因 / 处理 |
| --- | --- |
| 一张都检测不到 | 十有八九 `--cols/--rows` 填成格子数了；另外检查白边是否被裁、是否过曝/糊掉 |
| RMS 两三个像素 | 看 `per_view_error.csv` 找最差那几张；多半是板子弯了、角点检错、打印尺寸不准 |
| 内参看着收敛但不准 | 姿态全是大正面 → 看终端里的倾角范围和 3×3 覆盖格数，补拍四角 + 倾斜 |
| 主点跑到图像外 | 姿态覆盖不够或坏图太多；临时加 `--fix-principal` 稳住，但根本还是要补数据 |
| `--camera` 直接崩 | WSL 无 X 转发时开不了窗口，改用 `--images` / `--video` / `--synthetic` |
| 提示分辨率不一致 | 所有图必须同一分辨率，程序会自动跳过不一致的并打印提示 |
| 鱼眼/超广角镜头 | 本程序是普通针孔 + 多项式畸变模型，鱼眼要换 `cv::fisheye::calibrate` 和 4 参数 `k1~k4`；fixed 版给的内参也是普通模型 |

---

## 九、设计取舍

- **不依赖 Eigen**：标定全程只用 `cv::Mat`，避免为了一个独立小程序拖上 fixed 版的整套依赖。
- **不复用 fixed 版的视频/装甲板数据**：装甲板给出的是 4 个点，只有 8 个约束、且尺寸随
  `big` 变化，做内参标定远不如棋盘格稳；棋盘格一张图就有几十个共面点。
- **保留 `config/reference_intrinsics.yml`**：既是对照基准，也是 `--synthetic` 的真值来源；
  文件读不到时程序会用 `fixedFallback()` 里内置的那组数继续跑，不会因此挂掉。
- **从工程根目录运行**：相对路径 `config/`、`output/`、`images/` 都相对于当前目录。
