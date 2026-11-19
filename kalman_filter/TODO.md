# TODO

## 关联门限（待讨论）

10 rad/s 转靶下 `associate()` 的门限偏紧，一旦连续丢 4~5 帧（60 fps 下约 70 ms）预测就落后
45°，`max_yaw_innovation` 直接把它判掉，之后再也接不回来。要讨论的点：

- `max_yaw_innovation = 45°`：高速自旋时按预测的角速度自适应放宽，或者改成"按剩余不确定性
  放大"而不是固定角度。
- `chi2_gate = 9.49`（4 维 95%）：斜视板的 R 放大后 S 也跟着大，同一帧内更容易被错 id 吃掉；
  是不是该改成对 id 变化做惩罚（id 跳变记一次代价），而不是纯马氏距离比大小。
- `assoc_pos_gate = 2.5 m`：斜视板的坏位姿可以差 0.5 m，2.5 m 的门等于没有；按距离缩放，
  比如 `max(0.3 m, 0.1·dist)`。

## r 的真值不可测（待定）

`init()` 把半径写死 0.254（`kalman_filter.cc:199`），现场量不到真值，队长也不让量。
现在 `p_r=1e-6` + `init_sigma_r=5 mm` 让 r 几乎动不了，等于把这个错值当真值用，
剩下的残差只能由车心吸收——这是 px/pz 长期漂移的另一半来源。可选方案：

- 放开 r：`init_sigma_r` 给 2~3 cm、`p_r` 放到 1e-3 量级，让前几秒（转几十圈）就收敛，之后再收紧；
- 或者前 N 帧只估 r 和 yaw、车心钉死，稳定后再放开；
- 保守起见先给一个更小的初值（比如 0.20），让车心少背一点。

## 其它

- 起转瞬间（静止 → 10 rad/s）车心会被平移吃掉一部分：t≈15 s 起转时 pz 一次跳了 +0.28 m。
  纯靠位置观测跟上第一次突变，滤波器会优先平移。可考虑给 yaw 一个更大的初始 Q，
  或者在检测到 vyaw 突变时只放开 yaw 那一维。

## 设备戳是名义时钟（已定案，实现中）

### 结论

`nDevTimeStamp` 的相邻差恒为 **1,667,091 tick**（±1 tick 取整），换帧率才变，换实际帧率不变。
它是 `帧号 × (1/设定帧率)`，不是自由计数器 —— 实测同一常数 1,667,091 配过 16.65 ms 和 18.06 ms
两种真实周期。

推论：

- `tick_ns = 10.0` 只在相机真跑满 60 fps 时成立；掉到 56 fps 就等效 10.71，lag 按 6.66%/s 涨。
- 漂移到 1 s 被 `mvs_camera.cc` 的回退分支静音（`stamp` 换成 `host_time`），后面的延迟指标全部失真。
- 相机静止时无害（图旧但姿态也旧），相机一动就 `R_C0C` 配错时刻，整条链路崩。
- U3V 上 `nSecondCount/nCycleCount/nCycleOffset` 全 0，`GevTimestampTickFrequency` 节点不存在，
  SDK 没有第二个时间源可用。

### 任务

- [x] 新增 `src/sensors/device_clock.hpp/.cc`：纯类，零 SDK 依赖
  - `push(dev_ticks, host_seconds)`，环形缓冲，上限 300 对（≈5 s @60 fps）
  - 最少 45 对才出结果（≈0.75 s，即"第二版"的固定窗口，只是不冻结）
  - `fit()`：两轮 OLS，中间用 MAD 剔除单边离群（卡顿只会让延迟变大）
  - 返回 `{scale, offset, residual_ms, samples}`
- [x] 平滑与重锚
  - `a = a + 0.02 * (a_fit - a)`，时间常数约 50 帧
  - 换斜率时 `offset += now_ticks * (a_old - a_new)`，保证当帧 `stamp` 不跳
  - offset 用窗口内 `min(host - a*dev)`，每帧限幅 0.5 ms
- [x] 护栏
  - `lag > 0.5 s` 冻结（阈值不能太小，收敛期滞后会自己涨到 ~90 ms）
  - `|a_fit/target - 1| > 3%` 丢弃本次（和上次被接受的拟合值比，不是和平滑值比，
    否则第一次更新后就被自己锁死）
  - `a` 夹到 [9.0, 12.0] ns/tick
  - 窗口 dev 跨度 < 0.5 s 不拟合
  - 连续 5 次残差 RMS > 10 ms 判定模型失效，打警告并退回 host 时间
- [x] 接入 `mvs_camera.cc`
  - 每帧 push，取回 scale/offset 写进 `clock_scale_` / `clock_offset_`
  - `calibrateDeviceClock` 只保留 offset 的冷启动，scale 交给拟合器
- [ ] 把 `dev_us/dt_us` 的比值做成健康检查，偏离 1 超过 0.5% 报警（现在只打印不判断）
- [x] `config.hpp` + `cli.cc`：`--camera-clock-fit on|off`（默认 on）；
      `--camera-tick-ns` 仍是硬覆盖，给了就关拟合
- [x] 新增 `tests/device_clock_test.cpp`

### 验收

1. [x] `device_clock_test` 通过（离线合成序列：dev 恒增、host 17.86 ms ± 2 ms、插一次 200 ms 卡顿）
2. [ ] 实机跑 60 s，`[帧]` 的 `dev_us` 与 `dt_us` 稳定在 1% 以内
3. [ ] `--camera-fps 62`（相机只能到 60）跑一次，lag 保持恒定不涨
4. [ ] `[相机时钟]` 打出 `在线拟合 ns/tick=10.71`，和先验 10.00 对比
