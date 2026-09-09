# drone.yaml 参数使用分析

> 逐参数分析 `configs/drone.yaml` 中每个字段是否被使用、作用是什么、如何调整。
> 基于 `src/rbnx_auto_aim_debug.cpp` (无人机 Jetson 主程序) 的调用链路分析。

---

## 总览

| 统计 | 数量 |
|------|:---:|
| 总参数行 | 145 行 |
| ✅ 被无人机主程序使用 | ~47 个 |
| ⚠️ 其他模块使用(无人机未实例化) | ~17 个 |
| ❌ 未被使用 | ~22 个 |
| 🔴 缺失（会导致 `exit(1)` 崩溃） | **4 个** |

---

## 重要提示

- 无人机主程序 (`rbnx_auto_aim_debug.cpp`) 实例化的模块: **Gimbal, Camera(hikrobot), YOLO(trt_0526), Solver, Tracker, Aimer, Shooter, Planner**
- 无人机主程序 **未** 实例化的模块: CBoard(用Gimbal代替), Decider, USBCamera, Detector(传统检测器), Classifier(传统分类器)
- 编译模式: `TENSOR_RT_MAKE=ON`, `detector.cpp` 和 `classifier.cpp` **不参与编译**

---

## 一、✅ 被使用的参数（按模块）

### 1. 基本配置

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 2 | `enemy_color` | `tracker.cpp:23` `decider.cpp:24` | 敌方颜色 `"blue"/"red"/"auto"`。`"auto"` 时从电控模式推断；无人机固定 `"blue"` | 改为己方颜色；`"auto"` 让电控自动判断 |

### 2. 神经网络参数

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 7 | `yolo_name` | `yolo.cpp:22` | `"trt_0526"` → TensorRT YOLO 0526 引擎，Jetson 平台专用 | 其他选项: `trt_0708`(另一个TRT模型, 需对应engine文件) |
| 13 | `trt_engine_path_0526` | `trt_yolo_0526.cpp:280` | TRT 0526 engine 文件路径（**当前无人机使用此模型**） | 模型文件更新时修改 |
| 15 | `min_confidence` | `trt_yolo_0526.cpp:282` `tracker.cpp` | 检测置信度下限 (0~1)，低于此值的检测结果被丢弃 | 降低 → 更多检测但更多误检；升高 → 更少误检但可能漏检 |

### 3. 工业相机参数 (hikrobot)

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 50 | `camera_name` | `camera.cpp:18` | 相机品牌 `"hikrobot"` —— 硬件抽象层根据此名选择海康驱动 | 不改 |
| 51 | `camera_sn` | `camera.cpp:43` | 海康相机序列号 (如 `"DA7188471"`)，多相机时用于区分设备 | 更换相机时修改 |
| 52 | `exposure_us` | `camera.cpp:19` | 曝光时间(微秒)，影响画面亮度 | 室外增/室内减(范围 ~2000-12000) |
| 53 | `gain` | `camera.cpp:41` | 增益 (hikrobot: 0~1)，硬件放大信号 | 增大补光但增加噪点 |
| 56 | `flip` | `camera.cpp:23` → `hikrobot.cpp:210` | 图像垂直翻转，相机倒装时开启 | 画面上下颠倒时设为 `true` |
| 57 | `mirror` | `camera.cpp:24` → `hikrobot.cpp:213` | 图像水平镜像，相机反装时开启 | 画面左右颠倒时设为 `true` |
| 58 | `timestamp_offset_us` | `camera.cpp:20` | 时间戳偏移补偿(微秒)，用于对齐相机快门与陀螺仪时钟 | 调大 → 时间戳后移 |

### 4. CBoard / CAN 通信参数

> **注意**: 无人机主程序使用 `Gimbal` 而非 `CBoard` 进行通信(IMU通过Gimbal串口获取)。但以下参数被 `cboard.cpp` 读取，若其他程序实例化 CBoard 则会使用。

| 行 | 参数 | 读取位置 | 作用 |
|:---:|------|------|------|
| 79 | `quaternion_canid` | `cboard.cpp:110` | 四元数(IMU姿态)数据的 CAN ID `0x01` |
| 80 | `bullet_speed_canid` | `cboard.cpp:111` | 弹速/模式数据的 CAN ID `0x110` |
| 81 | `send_canid` | `cboard.cpp:112` | 视觉向电控下发指令的 CAN ID `0xff` |
| 82 | `can_interface` | `cboard.cpp:118` | CAN 接口名称 `"can0"` |

### 5. Tracker 参数

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 85 | `min_detect_count` | `tracker.cpp:25` | 连续检测 N 帧后才进入 TRACKING 状态 | 增大 → 更稳定但建立跟踪慢 |
| 86 | `max_temp_lost_count` | `tracker.cpp:26` | 临时丢失 N 帧后回 LOST 状态 | 增大 → 短暂遮挡不丢跟踪 |
| 87 | `outpost_max_temp_lost_count` | `tracker.cpp:27` | 前哨站专用丢失容忍度(距离远，值应更大) | 前哨站跟踪不够稳定时增大 |

### 6. Aimer 参数

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 90 | `yaw_offset` | `aimer.cpp:18` `planner.cpp:16` | yaw 方向静靶偏移补偿(度) | 子弹偏右 → **增大**；偏左 → **减小** |
| 91 | `pitch_offset` | `aimer.cpp:19` `planner.cpp:17` | pitch 方向静靶偏移补偿(度) | 子弹偏上 → **减小**；偏下 → **增大** |
| 92 | `comming_angle` | `aimer.cpp:20` | 小陀螺时"出现侧"角度阈值(度) | 通常 60°，不改 |
| 93 | `leaving_angle` | `aimer.cpp:21` | 小陀螺时"消失侧"角度阈值(度) | 通常 20°，不改 |
| 94 | `left_yaw_offset` | `aimer.cpp:25-27` | 左枪管的额外 yaw 偏移(度)，无人机双枪管专用 | 左枪管: 子弹偏右→减小,偏左→增大 |
| 95 | `right_yaw_offset` | `aimer.cpp:25-27` | 右枪管的额外 yaw 偏移(度)，无人机双枪管专用 | 右枪管: 子弹偏右→减小,偏左→增大 |
| 96 | `decision_speed` | `aimer.cpp:24` `planner.cpp:22` | 高低速分界角速度(rad/s)，低于此值用低速延迟，高于用高速 | 根据实际小陀螺转速调整 |
| 97 | `high_speed_delay_time` | `aimer.cpp:22` `planner.cpp:23` | 高速小陀螺的预测延迟(秒) | 子弹超调 → 减小；滞后 → 增大 |
| 98 | `low_speed_delay_time` | `aimer.cpp:23` `planner.cpp:24` | 低速平移的预测延迟(秒) | 同上 |
| 99 | `shoot_offset` | `planner.cpp:31` | MPC 开火决策的查询偏移步数（`shoot_offset * DT` = 查询 `t_fire` 时刻） | 不改(默认2) |

### 7. Shooter 参数

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 103 | `first_tolerance` | `shooter.cpp:13` | 近距离射击容差(度) | 增大 → 更容易开火但精度降低 |
| 104 | `second_tolerance` | `shooter.cpp:14` | 远距离射击容差(度) | 远距离应更严格(值更小) |
| 105 | `judge_distance` | `shooter.cpp:15` | 近/远距离分界阈值(米)，超过此距离用 `second_tolerance` | 根据弹道特性调整 |
| 106 | `auto_fire` | `shooter.cpp:16` | 是否由自瞄自动控制射击 | `false` → 手动开火 |

### 8. Planner 参数

| 行 | 参数 | 读取位置 | 作用 | 如何调整 |
|:---:|------|------|------|------|
| 109 | `fire_thresh` | `planner.cpp:21` | 轨迹残差开火阈值 | 增大 → 更容易开火 |
| 110 | `target_dist_error` | `planner.cpp:19` | 距离补偿(米)，修正测距系统偏差 | 实测距离偏小 → 增大 |
| 111 | `target_h_error` | `planner.cpp:20` | 高度补偿(米)，修正测高系统偏差 | 实测高度偏低 → 增大 |
| 112 | `small_armor_tolerance` | `planner.cpp:25` | 小装甲板开火窗口(rad)，`0.12 rad ≈ 6.9°` | 见下方调参指南 |
| 113 | `big_armor_tolerance` | `planner.cpp:26` | 大装甲板开火窗口(rad)，`0.22 rad ≈ 12.6°` | 见下方调参指南 |
| 114 | `gimbal_control_delay` | `planner.cpp:28` | 云台控制延迟(秒)，用于开火判断的时间回退 | 有动力学 → 0；无动力学 → 调大 |
| 115 | `tower_pitch_prediction_time` | `planner.cpp:29` | 前哨站 pitch 预测时间(秒) | 不改(默认0.05) |
| 117 | `max_yaw_acc` | `planner.cpp:587` | yaw MPC 角加速度上限(rad/s²) | **核心参数**，见下方调参指南 |
| 118 | `Q_yaw` | `planner.cpp:588` | yaw 跟踪代价权重 | 不改 `[9e6, 0]` |
| 119 | `R_yaw` | `planner.cpp:589` | yaw 平滑代价权重 | 不改 `[1]` |
| 121 | `max_pitch_acc` | `planner.cpp:610` | pitch MPC 角加速度上限(rad/s²) | 通常 > `max_yaw_acc` |
| 122 | `Q_pitch` | `planner.cpp:611` | pitch 跟踪代价权重 | 不改 `[9e6, 0]` |
| 123 | `R_pitch` | `planner.cpp:612` | pitch 平滑代价权重 | 不改 `[1]` |

### 9. 标定参数

| 行 | 参数 | 读取位置 | 作用 |
|:---:|------|------|------|
| 129 | `R_gimbal2imubody` | `solver.cpp:31` | IMU安装方向到云台的旋转矩阵 (3x3, 行优先) |
| 132 | `camera_matrix` | `solver.cpp:38` | 相机内参矩阵 3x3 (fx, 0, cx; 0, fy, cy; 0, 0, 1) |
| 133 | `distort_coeffs` | `solver.cpp:39` | 畸变系数 1x5 (k1, k2, p1, p2, k3) |
| 136 | `R_camera2gimbal` | `solver.cpp:32` | 手眼标定旋转矩阵 3x3 |
| 137 | `t_camera2gimbal` | `solver.cpp:33` | 手眼标定平移向量(米) |

### 10. 云台通信参数

| 行 | 参数 | 读取位置 | 作用 |
|:---:|------|------|------|
| 140 | `com_port` | `gimbal.cpp:14` | 串口设备路径，如 `"/dev/ttyUSB0"` |
| 143 | `gimbal_y1` | `gimbal.cpp:16` | yaw 轴映射编号: `±1/±2/±3`，负号表示取反，绝对值对应原始姿态的 y/p/r 序号 |
| 144 | `gimbal_p2` | `gimbal.cpp:17` | pitch 轴映射编号 |
| 145 | `gimbal_r3` | `gimbal.cpp:18` | roll 轴映射编号 |

---

## 二、⚠️ 其他模块使用但无人机主程序未实例化

> 以下参数代码中存在读取逻辑，但无人机主程序(`rbnx_auto_aim_debug.cpp`) **未实例化**对应模块。

### 全向感知模块 (omniperception::Decider) —— 无人机未使用

| 行 | 参数 | 读取位置 | 作用 |
|:---:|------|------|------|
| 28 | `image_width` | `decider.cpp:18` | 全向相机图像宽度 |
| 29 | `image_height` | `decider.cpp:19` | 全向相机图像高度 |
| 31 | `fov_h` | `decider.cpp:20` | 全向相机水平 FOV (度) |
| 32 | `fov_v` | `decider.cpp:21` | 全向相机垂直 FOV (度) |
| 35 | `new_fov_h` | `decider.cpp:22` | 校正后水平 FOV (度) |
| 36 | `new_fov_v` | `decider.cpp:23` | 校正后垂直 FOV (度) |
| 126 | `mode` | `decider.cpp:27` | 全向感知优先级模式(1/2/3) |

### USB相机模块 —— 无人机使用hikrobot

| 行 | 参数 | 读取位置 | 作用 |
|:---:|------|------|------|
| 37 | `usb_frame_rate` | `usbcamera.cpp:19` | USB相机帧率 |
| 38 | `usb_exposure` | `usbcamera.cpp:18` | USB相机曝光值 |
| 39 | `usb_gamma` | `usbcamera.cpp:20` | USB相机 gamma 值 |
| 40 | `usb_gain` | `usbcamera.cpp:21` | USB相机增益值 |

### 数字分类器 (rv_detector) —— 无人机使用TRT端到端

| 行 | 参数 | 读取位置 | 作用 |
|:---:|------|------|------|
| 72-76 | `number_classifier` 板块 | `rv_detector.cpp:21-39` | 装甲板数字分类器 (模型/标签/阈值/忽略类别) |

---

## 三、❌ 未被使用的参数

### 3.1 因 `TENSOR_RT_MAKE=ON` 不编译 (detector.cpp / classifier.cpp)

| 行 | 参数 | 原因 |
|:---:|------|------|
| 8 | `classify_model: ../assets/tiny_resnet.onnx` | `classifier.cpp` 在 TRT 编译时不参与编译。TRT YOLO 端到端推理，不需要额外分类器 |
| 16 | `use_traditional: true` | 传统检测器开关，`detector.cpp` 不参与编译 |
| 61 | `threshold: 150` | 同上 |
| 62 | `max_angle_error: 45` | 同上 |
| 63 | `min_lightbar_ratio: 1.5` | 同上 |
| 64 | `max_lightbar_ratio: 20` | 同上 |
| 65 | `min_lightbar_length: 8` | 同上 |
| 66 | `min_armor_ratio: 1` | 同上 |
| 67 | `max_armor_ratio: 5` | 同上 |
| 68 | `max_side_ratio: 1.5` | 同上 |
| 69 | `max_rectangular_error: 25` | 同上 |
| 70 | `min_confidence: 0.8` | 同上（注意：第15行也有 `min_confidence`，第15行的被 tracker 和 trt 使用） |

### 3.2 因模型选择 (drone 使用 `trt_0526`，不是其他模型)

| 行 | 参数 | 原因 |
|:---:|------|------|
| 9 | `yolo11_model_path` | 仅 `yolo_name == "yolo11"` 时读取 |
| 10 | `yolov8_model_path` | 仅 `yolo_name == "yolov8"` 时读取 |
| 11 | `yolov5_model_path` | 仅 `yolo_name == "yolov5"` 时读取 |
| 12 | `trt_engine_path_0708` | 仅 `yolo_name == "trt_0708"` 时读取 |
| 14 | `device: CPU` | TensorRT 强制使用 GPU，`"CPU"` 值仅在 `trt_0708` 中读取为 `"cuda:0"` 默认值 |

### 3.3 因相机品牌选择 (drone 使用 hikrobot，非 daheng/mindvision)

| 行 | 参数 | 原因 |
|:---:|------|------|
| 54 | `gamma: 0.7` | 仅 `camera_name == "daheng"` 或 `"mindvision"` 时读取，hikrobot 不读此键 |
| 55 | `vid_pid: "2bdf:0001"` | 仅 `camera_name == "mindvision"` 时从 yaml 读取；hikrobot 的 vid_pid 在 `camera.cpp:42` 硬编码为 `"2bdf:0001"` |

### 3.4 仅 trt_0708 读取而 drone 使用 trt_0526

| 行 | 参数 | 原因 |
|:---:|------|------|
| 19-23 | `roi` (区块) | `trt_yolo_0526.cpp` 构造函数**不读取** roi，仅 `trt_yolo_0708.cpp:485-493` 读取 |
| 25 | `use_roi: false` | 同上，`trt_yolo_0526` 不读取 |

---

## 四、🔴 缺少的必要参数（会导致程序崩溃）

以下参数被 `tools::read()` 强制读取，yaml 中缺少会导致 `exit(1)` 崩溃：

| 缺失参数 | 读取位置 | 作用 | 建议值 | 建议添加位置 |
|---------|------|------|:---:|-----|
| `img_gamma` | `camera.cpp:26` | Gamma 校正值，1.0 表示不校正画面 | `1.0` | 工业相机参数板块 (第58行后) |
| `far_pitch_offset` | `planner.cpp:18` | 远距离 pitch 补偿(度)，目标高度 > 1m 时使用此偏移而非 `pitch_offset` | `1.7` | planner 板块 (第115行后) |
| `tower_and_base_armor_tolerance_` | `planner.cpp:27` | 前哨站/基地开火窗口(rad)，`0.10 rad ≈ 5.7°` | `0.10` | planner 板块 (第115行后) |
| `gimbal_delay` | `planner.cpp:30` (`planner.hpp:56` 中使用) | 云台总延迟(秒)，= 系统延迟 + 电控延迟，用于延迟预测 | `0.002` | planner 板块 (第115行后) |

> **修复方法**: 在 `drone.yaml` 相应板块添加以上4个参数即可避免启动崩溃。

---

## 五、关键参数调参指南

### 命中率相关

| 症状 | 调整 |
|------|------|
| 子弹总是偏右 | `yaw_offset` **减小** |
| 子弹总是偏左 | `yaw_offset` **增大** |
| 子弹总是偏上 | `pitch_offset` **增大** |
| 子弹总是偏下 | `pitch_offset` **减小** |
| 小陀螺时打不中 | `high_speed_delay_time` 调大/调小，观察超调方向 |
| 平移时打不中 | `low_speed_delay_time` 调整 |
| 远距离 pitch 不准 | 调整 `far_pitch_offset`（需先添加此参数） |

### 跟踪/开火相关

| 症状 | 调整 |
|------|------|
| 快速小陀螺跟不上 | `max_yaw_acc` **增大**（不超过云台物理极限） |
| 云台抖动/超调 | `max_yaw_acc` **减小** |
| gimbal 滞后于 plan | `gimbal_control_delay` 和 `gimbal_delay` **增大** |
| 开火太频繁 / 命中率低 | `small_armor_tolerance` / `big_armor_tolerance` **减小** |
| 不开火 | `small_armor_tolerance` **增大**；`fire_thresh` **增大** |
| 跟踪容易丢 | `max_temp_lost_count` **增大** |
| 切换目标太频繁 | `min_detect_count` **增大** |
| 前哨站跟踪不稳 | `outpost_max_temp_lost_count` **增大** |
| 前哨站/基地不开火 | 调整 `tower_and_base_armor_tolerance_`（需先添加此参数） |

### 画面质量

| 症状 | 调整 |
|------|------|
| 画面太暗 / 远处灯条看不清 | `exposure_us` **增大** (2000~12000) |
| 画面过曝 / 灯条区域全白 | `exposure_us` **减小** |
| 画面噪点多 | `gain` **减小** (0~1) |
| 画面上下颠倒 | `flip: true` |
| 画面左右颠倒 | `mirror: true` |

---

## 六、快速修复建议

在 `drone.yaml` 中添加以下四行即可避免启动崩溃：

```yaml
# ===== 工业相机参数板块 (第58行之后) 添加 =====
img_gamma: 1.0               # Gamma校正(1.0=不校正)

# ===== planner 板块 (第115行之后) 添加 =====
far_pitch_offset: 1.7        # 远距离pitch补偿(度)
tower_and_base_armor_tolerance_: 0.10  # 前哨站/基地开火窗口(rad)
gimbal_delay: 0.002          # 云台总延迟(秒)
```

---

## 附录: 无人机主程序调用链路

```
main() -> rbnx_auto_aim_debug.cpp
 ├── io::Gimbal(config_path)      → com_port, gimbal_y1/p2/r3
 ├── io::Camera(config_path)      → camera_name, exposure_us, timestamp_offset_us, flip, mirror, img_gamma, gain, camera_sn
 ├── auto_aim::YOLO(config_path)  → yolo_name → trt_0526 → trt_engine_path_0526, min_confidence
 ├── auto_aim::Solver(config_path)→ R_gimbal2imubody, camera_matrix, distort_coeffs, R_camera2gimbal, t_camera2gimbal
 ├── auto_aim::Tracker(config_path)→ enemy_color, min_detect_count, max_temp_lost_count, outpost_max_temp_lost_count
 ├── auto_aim::Aimer(config_path) → yaw_offset, pitch_offset, comming_angle, leaving_angle, left/right_yaw_offset, decision_speed, high/low_speed_delay_time
 ├── auto_aim::Shooter(config_path)→ first_tolerance, second_tolerance, judge_distance, auto_fire
 └── auto_aim::Planner(config_path)→ yaw_offset, pitch_offset, far_pitch_offset, gimbal_delay,
                                      decision_speed, high/low_speed_delay_time, 
                                      target_dist_error, target_h_error, fire_thresh, shoot_offset,
                                      small/big_armor_tolerance, tower_and_base_armor_tolerance_,
                                      gimbal_control_delay, tower_pitch_prediction_time,
                                      max_yaw_acc, Q_yaw, R_yaw, max_pitch_acc, Q_pitch, R_pitch
```

> **未实例化的模块**: CBoard(CAN通信), Decider(全向感知), USBCamera(USB相机), Detector(传统检测器), Classifier(分类器)
