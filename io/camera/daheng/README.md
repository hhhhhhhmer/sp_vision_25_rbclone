# Daheng Camera

## 主要函数

- `DahengCamera::initSDK()`：初始化大恒 Galaxy SDK。
- `enum_and_check_camera()`：枚举设备并按序列号确认目标相机存在。
- `open_camera()` / `initialize_camera()`：打开设备并配置曝光、增益、Gamma 等参数（像素格式只做 `GXGetEnum` 读取，不写回）。
- `getFrame()`：从 SDK 抓一帧，内部已调用 `ProcessData()`，返回可直接使用的 `CV_8UC3` BGR 图像（深拷贝，避免多相机 imshow 出现横杠）。
- `ProcessData()`：完成 Bayer/Mono 转换、翻转、镜像和 OpenCV 图像构造。
- `read()` / `try_read()`：从软件队列读取带时间戳图像。
- `pause()` / `resume()`：控制采集线程暂停和恢复。

## 原理

采集线程不断从 Galaxy SDK 拉取原始帧，按像素格式转换为 BGR，并将图像及采集时刻放入队列
（时间戳在转换之后记录，因此对应转换完成时刻而非曝光结束时刻）。守护线程负责发现采集异常后重新打开设备。
`include/` 是厂商 SDK 头文件，不属于项目算法代码。
