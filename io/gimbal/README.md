# Gimbal

## 当前串口接口 `Gimbal`

- `state()` / `state_at()`：在线程锁保护下复制最新 yaw、pitch、弹速、弹丸计数和敌方颜色；
  `state_at()` 同时返回该帧反馈的接收时间。
- `mode()`：返回空闲、自瞄、小符、大符或长焦模式。
- `q(t)`：从姿态队列取出图像时间前后的四元数，用球面线性插值得到图像时刻姿态。
- `send(control, fire, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc)`：组装普通自瞄帧并发送。
- `send(VisionToGimbal)`：直接发送组装好的结构体；8 参数版本最终也汇入这条路径。
- `sb_send()`：发送附带目标位置和类别的哨兵帧。
- `omni_send()`：发送全向感知的 yaw、pitch 和距离。
- `open_serial()`、`reconnect()`：在多个候选串口中尝试连接，并在掉线后恢复。

`GimbalToVision`、`VisionToGimbal` 等 packed 结构直接定义线上的字节布局。接收线程校验帧头和 CRC，
解析状态并把姿态与接收时间放入队列。发送帧一律用局部变量组装（`write_frame()`），避免自瞄和
打符线程在模式切换瞬间同时写共享缓冲区、发出“自瞄 yaw + 打符 pitch”的混合帧。

注意 `sb_VisionToGimbal` 和 `OmniVisionToGimbal` 结构体里没有 CRC 字段，只有 `VisionToGimbal`
带 `crc16`。

## 旧 CAN 接口 `CBoard`

- `imu_at(timestamp)`：在相邻 CAN 四元数之间做 SLERP 插值。
- `send(Command)`：把控制、开火、yaw、pitch 和距离编码到 8 字节 CAN 帧。
- `callback(frame)`：按 CAN ID 解析四元数、弹速、模式和射击侧。
- `SocketCAN::write()`：发送 Linux CAN 帧；内部用 epoll 接收并调用回调。

## 原理和修改注意

相机曝光与姿态接收不是同一时刻，因此必须通过时间戳插值获得正确姿态。四元数使用 SLERP，避免直接对欧拉角线性插值造成跳变。

修改协议时必须同步核对下位机结构体、字段单位、大小端、packed 大小、帧头、帧尾和 CRC。

`VisionToGimbal` 的发送路径**已经计算 CRC**：`Gimbal::send()` 用 `tools::get_crc16()` 覆盖前 26 字节
（不含 `crc16` 字段与末尾 `end`），CRC 以小端放在字段 26..27。下位机上行帧的 CRC 位置与覆盖长度和
下行不同（上行 CRC 在 27..28、覆盖 0..26，与 `tools::check_crc16` 的读法自洽），改动帧布局时必须
两侧同步，否则表现为“所有帧被静默丢弃”。
