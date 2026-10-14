# USB Camera

## 主要函数

- `USBCamera(open_name, config_path)`：通过设备路径和 YAML 参数启动 OpenCV `VideoCapture`。
- `read()`：阻塞返回一帧图像。
- `read(img, timestamp)`：阻塞返回图像和采集时间戳。
- `read_for(..., timeout)`：在指定超时时间内等待一帧。
- 设备打开、失败重试和关闭是私有的 `open()` / `try_open()` / `close()`，由构造、析构和守护线程内部调用，外部只用上面的读取接口。

## 原理

采集线程持续调用 `VideoCapture::read()` 并将帧放入队列，守护线程监控设备状态并在掉线后重开。相比工业相机 SDK，它更通用，但曝光时刻和缓存控制通常没有工业相机精确，主要用于普通摄像头和调试。
