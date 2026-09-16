# Tools

## 作用

这里放被多个模块复用的基础工具。工具层不应依赖具体自瞄任务；只服务于识别、跟踪或瞄准的代码应放回对应任务目录。

## 数学、估计与控制

### `math_tools.*`

- `limit_rad(angle)`：把角度归一化到一个周期内，处理 yaw 差值时必须使用。
- `toeuler()`、`eulers()`、`rotation_matrix()`：在四元数、旋转矩阵和欧拉角之间转换。
- `xyz2ypd()` / `ypd2xyz()`：在笛卡尔坐标和 `[yaw,pitch,distance]` 之间转换。
- `xyz2ypd_jacobian()` / `ypd2xyz_jacobian()`：给 EKF 提供坐标变换雅可比矩阵。
- `delta_time()`：计算两个单调时钟时间点的秒数差。

### `extended_kalman_filter.*`

- `predict(F, Q)`：线性状态转移预测。
- 非线性 `predict(f)`：通过状态函数执行 EKF 预测。
- `update()`：根据观测、观测模型、雅可比和协方差校正状态。

EKF 用状态模型预测未来，再根据观测不确定度修正预测。角度等非线性状态需要调用者提供正确的状态加法和残差函数。

### `trajectory.*`

- `TrajectoryV1(v0, d, h)`：计算不考虑空气阻力的弹道角和飞行时间。
- `TrajectoryV2(v0, d, h)`：加入空气阻力修正；项目中的 `Trajectory` 默认指向该版本。

弹道求解根据初速、水平距离和高度计算 pitch。无物理解时设置 `unsolvable`，调用方不能继续使用结果。

### `pid.hpp`

- `PID::calc(set, fdb)`：根据目标值和反馈值计算 PID 输出，并应用积分和总输出限幅。

### `yaw_delay_model.*`

- `YawDelayModel::query(command_velocity, previous_velocity, now)`：按速度和方向插值云台延迟，换向窗口内加入惩罚。
- `direction()`、`reversal_active()`：提供延迟诊断状态。
- `estimate_yaw_delay(samples, ...)`：从命令/反馈记录中用相关性估计延迟。

### `fft.*` 和 `ransac_sine_fitter.*`

- `FFTExample::add_sample()`：加入带时间戳和可选装甲板编号的信号样本。
- `analyze()`：判断信号是否具有稳定周期，并估计频率、幅值、相位和均值。
- `get_wave()`：返回可用于预测值、速度和加速度的 `Wave`。
- `RansacSineFitter::fit()`：用 RANSAC 拒绝异常点后拟合正弦模型。

这组工具用于识别前哨站等周期运动。FFT 提供频率候选，RANSAC 提高对漏检和错误观测的鲁棒性。

## 并发工具

### `thread_safe_queue.hpp`

- `push()`：线程安全入队；模板参数可选择队满时覆盖最旧元素。
- `pop()` / `front()`：阻塞取得元素或队首。
- `try_pop()`、带超时读取：用于不希望永久等待的线程。
- `close()`：唤醒等待线程并阻止继续入队，程序退出时应优先使用。
- `clear()`、`size()`：清空和查看队列状态。

内部使用互斥量和条件变量实现生产者/消费者同步。队列容量需要按实时性选择：视觉实时流程通常宁可丢旧帧，也不要积累高延迟。

### `periodic_timer.hpp`

- `PeriodicTimer(period)` + `wait_next()`：按**绝对时刻**（`t0 + k·period`）唤醒循环，周期不再等于"工作耗时 + sleep"；单轮超时不补发，直接对齐到未来整节拍。
- `missed()`：因超时被跳过的节拍数，用于在线判断算力是否够。
- `tighten_timer_slack()`：收紧 Linux 线程定时器松弛（默认 50 us），减少 sleep 的系统性迟到。

以固定频率向下位机下发指令时必须用它替代 `std::this_thread::sleep_for`，详见 `docs/send_rate_stability.md`。

### `thread_pool.hpp`

- `ThreadPool(thread_count)`：创建固定数量工作线程。
- `enqueue(task)`：把一个无返回值任务加入工作队列。
- `~ThreadPool()`：停止接收任务，唤醒线程并等待工作线程退出。

## 调试、绘图与记录

### `plotter.*`

- `plot(json)`：通过 UDP 向 PlotJuggler 发送一组数据，并在启用时写入原始记录。
- `start_recording()` / `stop_recording()`：开始记录，并在结束时生成 CSV。
- `replay(path, speed)`：按原始时间间隔重新发送 CSV 或 JSONL。

### `recorder.*`

- `Recorder::record(img, q, timestamp, stream_id)`：把图像、姿态和时间戳异步写入视频及文本。

保存线程与主视觉线程分离。分辨率、像素类型或流 ID 改变时会打开新视频分段，避免 `VideoWriter` 写入不兼容帧。

### 图像辅助

- `draw_point()`、`draw_points()`、`draw_text()`：统一 OpenCV 调试绘制。
- `draw_reprojection()`：把 EKF 预测装甲板和瞄准点重新投影到图像，用于检查解算和跟踪是否一致。
- `fpsSolve::get_fps()` / `get_mean_fps()`：统计瞬时和滑动平均帧率。

## 运行与配置辅助

- `logger()`：返回全局 spdlog logger。
- `load(path)`、`read<T>(yaml, key)`：读取 YAML，并在文件或键缺失时统一报错退出。
- `Exiter::exit()`：通过信号安全地通知主循环退出。
- `SystemdWatchdog::ready()` / `ping()`：向 systemd 发送服务就绪和看门狗心跳。
- `get_crc8()`、`get_crc16()`：生成协议校验；`check_crc8()`、`check_crc16()` 用于验证接收帧。

## 修改原则

工具函数应尽量无业务状态、接口小且可独立测试。新增工具前先确认它会被多个模块使用；否则放到拥有该逻辑的功能目录更容易理解。
