# Omniperception

## 作用

全向感知负责"看不到敌人的方向也能发现敌人"：多路广角相机同时取图、并行推理，把各方向的检测结果
汇总成一份带方位的目标列表，供决策器选靶并直接生成云台命令（哨兵）或提供给自瞄跟踪器做融合。

```text
多路相机（USBCamera / Camera）-> Perceptron 并行推理 -> std::vector<DetectionResult>
    -> Decider -> io::Command / io::VisionToGimbal
    ->（可选）Tracker::track(detection_queue, armors, t) 融合进自瞄目标
```

本目录只处理"发现与选靶"，自瞄的三维跟踪仍在 `tasks/auto_aim/`。相机取图、云台通信和标定复用
`io/` 与既有模块。

## 主要类型

### `DetectionResult`

单路相机一帧的检测结果（`detection.hpp`）：`armors`、`timestamp`，以及装甲板相对相机光轴的
`delta_yaw` / `delta_pitch`（弧度）。全向相机不追求精确定位，只给出方位用于选靶。

### `Decider`

决策器，构造时读取机器人 YAML（`decider.cpp`）：`image_width`、`image_height`、`fov_h`、`fov_v`、
`new_fov_h`、`new_fov_v`、`enemy_color`、`mode`。其中 `image_width/image_height/fov_h/fov_v`
当前读取后不参与计算，真正用的是 `new_fov_*`（把归一化像素坐标换算成 yaw/pitch）和
`enemy_color`（值为 `auto` 时按云台反馈的敌方颜色动态取色）。

- `decide_g(yolo, gimbal_pos, cam_l, cam_r, left_solver, right_solver, target_distance)`：融合左右全向
  相机，返回哨兵用的 `io::VisionToGimbal` 帧。
- `decide(yolo, gimbal_pos, usbcam1, usbcam2, back_camera)` / `decide(yolo, gimbal_pos, back_camera)`：
  融合两路 USB 相机加后相机，或只用单路后相机，返回 `io::Command`。
- `decide(detection_queue)`：直接用已有的多路检测结果生成命令（不取图）。
- `delta_angle()` / `delta_angle_3d()`：前者按二维装甲板位置算相对光轴角度，后者借 `Solver` 的三维
  解算算角度，精度更高但依赖位姿求解。
- `armor_filter()` / `not_base_armor_filter()` / `set_priority()` / `sort()`：过滤、按优先级排序候选。
- `get_target_info()`、`get_invincible_armor()`、`get_auto_aim_target()`：对外提供选中的目标信息、
  无敌状态目标（英雄为 1、哨兵为 6）和需要转交自瞄的目标。
- `set_gimbal()`、`set_mode()`：注入云台（用于取敌方颜色和姿态）与切换优先级模式。

优先级由 `mode` 选择三张表之一（`decider.hpp` 的 `mode1`/`mode2`/`mode3`，对应
`PriorityMode::MODE_ONE/MODE_TWO/MODE_THREE`）：`mode1` 优先三/四号步兵，`mode2` 优先二号步兵，
`mode3` 优先前哨站；其余目标按梯队顺延。

### `Perceptron`

多路取图 + 并行推理的封装：

- `Perceptron(...)`：一路直接管理 4 路 USB 相机；另一路接收 `IFrameSource` 与 `IArmorDetector`，
  便于测试和替换相机/识别器。
- `get_detection_queue()`：取出本轮的 `std::vector<DetectionResult>`。
- `parallel_infer(...)` / `parallel_infer_source(...)`：每路相机一个线程，推理结果经
  `tools::ThreadSafeQueue` 汇总。

`IFrameSource`（`frame_source.hpp`）只要求 `read_for(image, timestamp, timeout)` 和 `name()`，
把"取图"从具体相机解耦。

## 编译与配置

`omniperception` 是 OBJECT 库，在顶层 CMake 中仅当后端为 OpenVINO 时加入构建
（`if(OPENVINO_MAKE) add_subdirectory(tasks/omniperception)`），并链接 `openvino::runtime`。

YAML 字段的完整说明见 `configs/README.md` 的"全向感知"一节。注意 `main_and_secondary` 由调用方使用，
当前有效逻辑只认 `left` 和 `right`。

## 运行入口与测试

- `src/omni_perception.cpp`：全向感知单独运行程序。
- `src/sb_omn_aim_mpc.cpp`、`src/sentry_omn_aim_mpc.cpp`、`src/sentry_binocular_aim_mpc.cpp`：
  哨兵/步兵整机程序，构造 `omniperception::Decider`。
- `tests/omniperception_test.cpp`、`tests/perceptron_stop_test.cpp`、`tests/safety_boundaries_test.cpp`。
