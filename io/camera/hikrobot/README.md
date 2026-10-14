# HikRobot Camera

## 主要函数

- `HikRobot(sn, exposure_us, gain, vid_pid, flip, mirror)`：按序列号、曝光、增益、USB VID:PID 和翻转配置创建相机（`vid_pid` 固定传 `2bdf:0001`，用于掉线复位）。
- `ChoiceCamrea()`：从海康 SDK 枚举结果中选择匹配序列号的设备。
- `capture_start()` / `capture_stop()`：启动或停止采集线程和 SDK 抓图。
- `read()` / `try_read()`：读取带采集时间戳的 BGR 图像。
- `clear_camera_frame_buffer()`：同时清空软件队列和海康 SDK 缓冲区。
- `set_float_value()` / `set_enum_value()`：设置 SDK 参数节点。
- `reset_usb()`：设备异常时复位对应 USB 设备。

## 原理

驱动用独立采集线程把 SDK 图像转换为 OpenCV 格式并入队；守护线程负责掉线后的恢复。每帧在进入队列时绑定时间戳，主线程消费速度慢时可清空缓存避免处理过期图像。

`include/` 和 `lib/` 分别是厂商头文件及不同架构的预编译库。修改驱动时保持 `CameraBase` 的读取和时间戳约定不变。
