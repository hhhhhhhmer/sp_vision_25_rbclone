# Camera

## 主要类和函数

### `CameraBase`

- `read(img, timestamp)`：阻塞等待一帧图像。
- `try_read(img, timestamp)`：立即尝试读取，成功时返回 `true`。
- `pause()` / `resume()`：暂停或恢复支持该能力的工业相机（迈德威视未重写，调用无效）。
- `clear_camera_frame_buffer()`：清除旧帧；目前只有海康会同时清 SDK 缓冲和软件队列，大恒、迈德威视是空实现。

所有工业相机实现都继承该接口，并维护采集状态、序列号和最近读取时间。注意迈德威视的序列号是硬编码的
`"nan"`（不读 YAML 的 `camera_sn`），而且三家的 `last_read_t` 都只在 `try_read()` 里更新。

### `Camera`

- `Camera(config_path)`：根据 YAML 的 `camera_name` 创建大恒、海康或迈德威视相机。
- `read()` / `try_read()`：转发到底层驱动，然后执行统一图像后处理。
- `initSDK()`：初始化需要全局初始化的相机 SDK（目前只初始化大恒）。
- `pause()`、`resume()`、`is_paused()`：统一控制底层采集状态。
- `get_last_read_t()`、`get_capturing()`、`get_camera_sn()`：查询底层相机状态。
- `timestamp_offset`：由 YAML 的 `timestamp_offset_us` 读入并保存，`read()`/`try_read()` **不会**自动应用它。目前 `src/` 下没有程序真正拿它补偿姿态查询（`rb_binocular_aim_debug_async.cpp` 取出了该值但未使用，`rb_binocular_aim_debug.cpp` 里的补偿代码是注释掉的），实际补偿靠 `gimbal.q(t - 3ms)` 这类手写偏移。
- 构造时无条件读取 `img_gamma`，因此三家的配置都必须包含该键，否则启动即报错退出。

### 图像处理

- `make_protected_gamma_lut(gamma, shadow_offset)`：创建 8 位 Gamma 查找表，并避免暗部被过度拉升。
- `apply_luma_protected_gamma()`：转到 YCrCb，只处理亮度通道，并分别对亮度、色度降噪。

## 原理

具体驱动在后台线程中从 SDK 取帧，记录单调时钟时间戳，再把 `cv::Mat + timestamp` 放入线程安全队列
（大恒是先做像素格式转换、再记录时间戳，因此时间戳对应转换完成时刻而非曝光结束时刻）。
统一 `Camera` 层不关心厂商句柄，只负责选择实现并做一致的图像增强；时间戳补偿由上层按需完成。

`USBCamera` 使用 OpenCV `VideoCapture`，是单独的通用相机类，目前不由 `Camera` 工厂创建，主要供全向感知和双目/多目测试程序使用。

修改采集流程时应保持 `CameraBase` 的时间戳和阻塞语义不变，否则图像与云台姿态可能无法正确对齐。
