# Tools

## 作用

这里放被多个模块复用的基础工具。工具层不应依赖具体自瞄任务；只服务于识别、跟踪或瞄准的代码应放回对应任务目录。

## 数学、估计与控制

### `math_tools.*`

- `limit_rad(angle)`：把角度归一化到 `(-π, π]`，处理 yaw 差值时必须使用。
- `toeuler()`、`eulers()`、`rotation_matrix()`：在四元数、旋转矩阵和欧拉角之间转换。
- `xyz2ypd()` / `ypd2xyz()`：在笛卡尔坐标和 `[yaw,pitch,distance]` 之间转换。
- `xyz2ypd_jacobian()` / `ypd2xyz_jacobian()`：给 EKF 提供坐标变换雅可比矩阵。
- `delta_time()`：计算两个单调时钟时间点的秒数差。
- `get_abs_angle(vec1, vec2)`：两个二维向量的无符号夹角（任一为零向量时返回 0）。
- `square<T>(a)`、`limit_min_max(input, min, max)`：平方与区间限幅小工具。

### `extended_kalman_filter.*`

- `predict(F, Q)`：线性状态转移预测。
- `predict(F, Q, f)`：非线性预测，`f` 是状态函数，`F` 仍需给出（用于状态转移，协方差按 `F` 传播）。
- `update(z, H, R)`：线性观测更新。
- `update(z, H, R, h, z_subtract)`：非线性观测更新，需显式给出观测函数 `h` 与残差函数 `z_subtract`。

EKF 用状态模型预测未来，再根据观测不确定度修正预测。角度等非线性状态需要调用者提供正确的状态加法和残差函数，两者分别通过构造函数的 `x_add` 与 `z_subtract` 注入。

`update()` 在创新协方差非正定时抛 `std::runtime_error`。滤波器内置 NIS/NEES 卡方诊断：`data`
（`residual_yaw/pitch/distance/angle`、`nis`、`nees`、`nis_fail`、`nees_fail`、`recent_nis_failures`）、
`get_Chi_Square_test_datas()`、`last_nis`，以及滑动窗口 `window_size`（默认 100）可读出结果，
`recent_nis_failures` 是最近失败标记的双端队列。

### `trajectory.*`

- `TrajectoryV1(v0, d, h)`：计算不考虑空气阻力的弹道角和飞行时间。
- `TrajectoryV2(v0, d, h)`：加入空气阻力修正；项目中的 `Trajectory` 默认指向该版本。

弹道求解根据初速、水平距离和高度计算 pitch。`TrajectoryV2` 按初速自动选弹丸参数（`v0 > 18 m/s`
用小弹 `kSmall`，否则用大弹 `kBig`，重力取 `g = 9.7833`）；判别式 < 0 或解出的 `|sinθ| > 1` 时
设置 `unsolvable`，此时 `pitch` 和 `fly_time` 都不可用，调用方不能继续使用结果。

### `pid.hpp`

- `PID(dt, kp, ki, kd, max_out, max_iout, angular = false)`：构造时给出控制周期与限幅；`angular = true` 时误差与微分项按 `limit_rad` 处理，适合角度量控制。
- `PID::calc(set, fdb)`：根据目标值和反馈值计算 PID 输出，并应用积分和总输出限幅。
- `pout` / `iout` / `dout`：保存分项输出，便于调试。

### `fft.*` 和 `ransac_sine_fitter.*`

- `FFTExample(max_points = 1200)`：设定缓冲区容量；`reset()` 清空样本，`low_pass(value, alpha = 0.05)` / `reset_low_pass()` 提供一阶低通辅助。
- `FFTExample::add_sample()`：加入带时间戳和可选装甲板编号的信号样本。
- `analyze(force = false)`：判断信号是否具有稳定周期，**返回本次是否确认周期**；频率、幅值、均值、拟合质量和信噪比分别用 `get_frequency()`、`get_amplitude()`、`get_mean_val()`、`get_fit_quality()`、`get_signal_to_noise_ratio()` 读取，相位用 `set_phase_offset()` 调整。
- `get_wave()`：返回可用于预测值、速度和加速度的 `Wave`。
- `RansacSineFitter(max_iterations, threshold, min_omega, max_omega)` + `add_data(t, v)` + `fit()`：用 RANSAC 拒绝异常点后拟合正弦模型，结果写入 `best_result_`（幅值 `A`、角频率 `omega`、相位 `phi`、偏置 `C`、内点数、RMS）。
- 辅助接口：`ready(min_samples, min_time_span, min_inlier_ratio, max_rms)`、`inlier_ratio()`、`sample_count()`、`time_span()`、`clear()`。

这组工具用于识别前哨站等周期运动。FFT 提供频率候选，RANSAC 提高对漏检和错误观测的鲁棒性。

## 并发工具

### `thread_safe_queue.hpp`

- `push()`：线程安全入队，返回 `QueuePushResult`（`Pushed` / `DroppedNewest` / `DroppedOldest` / `Closed`）。
- 构造时用 `ThreadSafeQueue(max_size, OverflowPolicy, full_handler)` 选择 `DropNewest` / `DropOldest` / `Block`
  （`max_size == 0` 抛 `std::invalid_argument`，`full_handler` 在溢出时回调）；`PopWhenFull` 模板参数是旧的简化写法。
- `pop()` / `front()`：阻塞取得元素或队首；在“已关闭且为空”时抛 `std::runtime_error`。
- `wait_pop()` / `wait_pop_for(timeout)`：不抛异常地等待，关闭或超时返回 `std::nullopt`。
- `try_pop()`、带超时读取：用于不希望永久等待的线程。
- `close()`：唤醒等待线程并阻止继续入队，程序退出时应优先使用。
- `back(T&)`、`clear()`、`size()`、`empty()`、`closed()`：查看队尾、清空和查询队列状态。

内部使用互斥量和条件变量实现生产者/消费者同步。队列容量需要按实时性选择：视觉实时流程通常宁可丢旧帧，也不要积累高延迟。

### `periodic_timer.hpp`

- `PeriodicTimer(period)` + `wait_next()`：按**绝对时刻**（`t0 + k·period`）唤醒循环，周期不再等于"工作耗时 + sleep"；单轮超时不补发，直接对齐到未来整节拍。`wait_next()` 返回本次唤醒相对节拍点的偏差，可直接喂给均值/标准差统计做在线抖动监控。
- `missed()`：因超时被跳过的节拍数，用于在线判断算力是否够。
- `period()`、`reset()`：读取周期；模式切换后把下一节拍点重新对齐到 `now + period`。
- `tighten_timer_slack(slack_ns = 1)`：收紧 Linux 线程定时器松弛（内核默认 50 us，本函数默认收窄到 1 ns），减少 sleep 的系统性迟到；非 Linux 平台为空实现。

以固定频率向下位机下发指令时必须用它替代 `std::this_thread::sleep_for`，否则工作耗时抖动会 1:1 传到发送间隔上。

### `thread_pool.hpp`

- `ThreadPool(thread_count)`：创建固定数量工作线程；`thread_count == 0` 抛 `std::invalid_argument`。
- `enqueue(task)`：把一个无返回值任务加入工作队列；线程池停止后再调用抛 `std::runtime_error`。
- `~ThreadPool()`：停止接收任务，唤醒线程并等待工作线程退出。

本文件是纯头文件，不参与 `tools` 库编译。

## 调试、绘图与记录

### `plotter.*`

- `Plotter(record = true, host = "127.0.0.1", port = 9870, record_path = "")`：构造即开始录制（`record = false` 只发 UDP）；默认保存到 `plot_records/<时间戳>.csv`，并保留同名 `.jsonl` 原始记录，两者成对出现。
- `plot(json)`：通过 UDP 向 PlotJuggler 发送一组数据，并在启用时写入原始记录。
- `start_recording(path)` / `stop_recording()`：开始记录，并在结束时生成 CSV；返回 `bool`，传 `.csv` 或 `.jsonl` 都会成对生成。
- `recording()` / `recording_path()`：查询是否在录制，以及当前/最近一次使用的记录路径。
- `replay(path, playback_speed = 1.0)`：按记录时间间隔重发。实际读取的是与 `path` 同名的 `.jsonl`（传 `.csv` 会被换成同名 `.jsonl`，只有 CSV 时回放直接失败），倍速必须为有限正数，要求记录时间戳单调，返回是否完整发送。

### `recorder.*`

- `Recorder(fps = 30)`：输出到 `records/<时间戳>.txt`（每行 `时间 四元数wxyz`）和分段视频 `records/<时间戳>_<stream_id>_NNN.avi`（MJPG）。
- `Recorder::record(img, q, timestamp, stream_id)`：把图像、姿态和时间戳异步写入视频及文本；会按 `fps` 节流丢帧，`stream_id` 为空时按 `default` 处理并替换非法字符。

保存线程与主视觉线程分离。分辨率、像素类型或流 ID 改变时会打开新视频分段，避免 `VideoWriter` 写入不兼容帧。

### 图像辅助

- `draw_point()`、`draw_points()`、`draw_text()`：统一 OpenCV 调试绘制；`draw_points()` 有 `cv::Point` 与 `cv::Point2f` 两个重载，颜色、半径/线宽、字号都有默认值。
- `draw_reprojection()`：把 EKF 预测装甲板和瞄准点重新投影到图像，用于检查解算和跟踪是否一致。
- `fpsSolve(window_size = 200)`：`update(timestamp)` 返回瞬时帧率，`get_fps()` / `get_mean_fps()` 取瞬时与滑动平均，`reset()` 清空统计。

### `uv_overlay.*`（UV 观测可视化）

- `draw_uv_overlay(img, target, armors, uv_mode, fused, nis, config)`：在图像上叠加 UV 观测结果——
  模型预测的全部装甲板轮廓（灰色=非当前板，绿色=当前 `last_id`）、每块检测装甲板的观测角点（青色）
  与模型预测角点（红色）及其残差连线、每条灯条的 UVL 残差数值；返回第一块匹配装甲板的外接矩形。
- `UvOverlayConfig`：`scale`、`font_scale`、`status_pos`、`text_base_y`、`text_line_step`、
  `max_shown`、`draw_all_armors`、`draw_residual_text`，适配缩放后画面的坐标与字号。
- `draw_zoom_inset(img, focus, size, scale, margin_px)`：把装甲板附近区域放大贴到右上角，
  方便肉眼看灯条端点级别的拟合。

只有 UV 模式（`target.ekf().uv_model().ready()`）才绘制，否则只画一行模式提示并返回空矩形。
注意依赖方向：`uv_overlay.cpp` 需要 `auto_aim` 的 `Target`/`UvModel`，所以在根 `CMakeLists.txt` 里
被编成独立的 `uv_overlay` OBJECT 库（而不是放进 `tools` 库），只有需要画 UV 的目标才链接它。

## 独立调试程序

- `plotter_replay.cpp`：`plotter_replay <csv|jsonl> [倍速] [IP] [端口]`，按原始时间间隔把记录
  重新发给 PlotJuggler，用于离线复现当时的数据。
- `gimbal_calib.cpp`：云台电机响应标定工具。先做 ±3°/±8°/±15° 阶跃拟合纯死区 τ 与一阶收敛
  时间常数 T_cl，再做 2/4/6 rad/s 匀速梯形波测稳态滞后得到等效延迟 D(ω)，最后打印可直接写进
  配置的 `gimbal_delay`（单位秒），并附带打印各轴 τ/T_cl 供人工评估（Planner 已不再读取 τ/T_cl）。
  用法 `gimbal_calib <config.yaml> [--out records.jsonl] [--self-test]`；`--self-test` 用合成的一阶+延迟
  对象自检拟合数学，采样记录写入 `--out`（默认 `gimbal_calib_records.jsonl`）。匀速段幅度受安全限幅
  （yaw 45°、pitch 10°），D(ω) 随速度变化超过 25% 时会提示“常数不适用、建议改用速度相关曲线表”。
  还要求车体静止、电控 PID 为实战值，反馈姿态由四元数推导（含约 5~10ms IMU 解算滞后，补偿过度时把
  `gimbal_delay` 减小对应毫秒数）。

## 云台延迟补偿现在在哪里

早期版本在工具层做过速度插值的延迟模型（`yaw_delay_model.*` + `yaw_delay_analyzer.cpp`），该提交
（`0ee04cb`）已被整体回退（`76b8bfa`），这些文件在仓库中不存在。现在没有插值模型，实际补偿是
Planner 内的固定分段延迟：

```text
delay = ( |ekf_x()[7]| > decision_speed ? high_speed_delay_time : low_speed_delay_time ) + gimbal_delay
```

按 `decision_speed` 分档，见 `tasks/auto_aim/aiming/planner/planner.hpp`，配置键的读取见
`planner.cpp`（`Aimer` 里也读同一组高/低速延迟）。`gimbal_delay` 的实测值由本目录的
`gimbal_calib` 产出。

## 运行与配置辅助

- `logger()`：返回全局 spdlog logger；首次调用时创建 `logs/<时间戳>.log` 文件 sink 和彩色控制台 sink（级别 debug，info 及以上立即 flush）。
- `load(path)`、`read<T>(yaml, key)`：读取 YAML，并在文件或键缺失时统一报错并 `exit(1)`（只适合主程序/测试进程使用）。
- `Exiter()` / `exit()`：构造时注册 SIGINT 处理器（重复构造第二个实例会抛 `std::runtime_error`）；主循环轮询 `bool exit() const` 判断是否收到退出信号。
- `SystemdWatchdog::ready()` / `ping()`：向 systemd 发送服务就绪和看门狗心跳。只有设置了 `NOTIFY_SOCKET` / `WATCHDOG_USEC`（且 `WATCHDOG_PID` 匹配）时才生效，心跳周期为 `WATCHDOG_USEC / 2`；未启用时 `ping()` 直接返回 true。另有 `stopping()`。
- `get_crc8()`、`get_crc16()`：生成协议校验；`check_crc8()`、`check_crc16()` 用于验证接收帧。注意生成函数的 `len` **不含**校验字节，校验函数的 `len` **含**校验字节（CRC-16 为末尾 2 字节，高字节在后）。

## 编译方式

`tools/CMakeLists.txt` 只把以下实现编进 `tools`（OBJECT 库）：`exiter.cpp`、
`extended_kalman_filter.cpp`、`ransac_sine_fitter.cpp`、`img_tools.cpp`、`math_tools.cpp`、
`plotter.cpp`、`trajectory.cpp`、`recorder.cpp`、`reprojection.cpp`、`logger.cpp`、`pid.cpp`、
`crc.cpp`、`fft.cpp`、`fps_solve.cpp`。

纯头文件、不参与编译：`thread_safe_queue.hpp`、`thread_pool.hpp`、`periodic_timer.hpp`、
`systemd_watchdog.hpp`、`yaml.hpp`。

`uv_overlay.cpp` 因依赖 `auto_aim` 而在根 `CMakeLists.txt` 里单独编成 `uv_overlay` OBJECT 库；
`gimbal_calib.cpp`、`plotter_replay.cpp` 各自编成可执行文件；`fft.cpp` 另外还被 `fft_fit_test` 直接编译。
新增工具时要注意：编进 `tools` 这个 OBJECT 库的目标文件会进入每个链接它的目标，不要引入
`auto_aim` 等上层依赖。

## 修改原则

工具函数应尽量无业务状态、接口小且可独立测试。新增工具前先确认它会被多个模块使用；否则放到拥有该逻辑的功能目录更容易理解。
