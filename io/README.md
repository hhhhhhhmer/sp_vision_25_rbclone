# IO

## 作用

把相机 SDK、串口、CAN 和 ROS2 等硬件细节包装成稳定接口。算法模块应该使用 `Camera`、`Gimbal`、`CBoard` 等类，不直接调用厂商 SDK。

## 子目录和主要入口

| 目录 | 主要入口 | 作用 |
| --- | --- | --- |
| `camera/` | `Camera::read()`、`Camera::try_read()` | 获取带时间戳图像并做统一后处理 |
| `gimbal/` | `Gimbal::state()`、`q()`、`send()` | 接收云台状态、对齐姿态并发送控制量（串口） |
| `serial/` | `serial::Serial::read()`、`write()` | 跨平台串口访问 |
| `dm_imu/` | `DM_IMU::imu_at()` | 读取独立 IMU 并按时间插值 |
| `ros2/` | `ROS2::publish()`、订阅接口 | 与导航进程交换目标和敌方状态 |

`gimbal/gimbal.*` 是当前主用的串口通信路径；`gimbal/cboard.*` 是 CAN 通信路径，仍被
`sentry_*_mpc`、`sb_omn_aim_mpc`、`omniperception_test`、`4cmaera_test`、`cameras_test`
以及 `tests/sp_example/` 下的示例程序使用，不是废弃代码。两者都提供图像时刻姿态和控制发送能力，
但协议不同。

`camera/usbcamera/` 是独立的 USB 相机类，不由 `Camera` 工厂创建，主要供全向感知
（`tasks/omniperception/`）和双目/多目测试程序直接使用。`io/serial/` 是 vendored 的
跨平台串口库，`io/ros2/` 只在编译时找到 ROS2 环境才会被编进 `io` 库。

## 原理

硬件接收通常在后台线程中持续运行，把图像或姿态连同 `steady_clock` 时间戳放进线程安全队列。主流程按图像时间查询对应姿态，避免直接使用“当前姿态”造成时间错位。发送侧把算法输出编码为固定协议帧，再交给串口或 CAN。

推荐先读 `camera/camera.hpp` 和 `gimbal/gimbal.hpp`，只有排查设备连接、掉线恢复或像素转换时再进入具体厂商目录。
