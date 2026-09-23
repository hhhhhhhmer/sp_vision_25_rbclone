# RP-26Rune 全链路接入说明

`auto_buff` 负责完整的能量机关处理链路，包括五点模型推理、语义轮廓精修、Chamfer 位姿
优化、大小符运动估计、弹道拦截和自动火控。相机取帧、IMU 姿态插值、云台通信及机器人
标定仍由当前项目原有模块负责。

## 公共接口

调用方只需包含 `tasks/auto_buff/rune_system.hpp`，使用机器人 YAML 配置构造一个
`auto_buff::RuneSystem`，然后在固定的图像线程中调用 `process()`。退出打符模式时必须调用
`reset()`，清除运动拟合、目标选择和开火冷却状态。

返回的 yaw、pitch 遵循当前项目的坐标约定，单位均为弧度。第一版不提供速度和加速度前馈，
调用 `gimbal.send()` 时对应字段保持为零。

`debug_snapshot()` 提供五点检测结果、语义轮廓、模型重投影、预测命中点、运动参数、阶段耗时
和失败关闭原因。算法核心不会创建或管理 GUI 窗口，调试程序使用当前项目的 OpenCV 和
PlotJuggler 工具显示这些数据。

## 配置规则

`config/power_rune_defaults.yaml` 包含完整的 RP 算法默认参数。机器人 YAML 中的以下现有标定
字段始终优先：

- `camera_matrix`、`distort_coeffs`
- `R_gimbal2imubody`
- `R_camera2gimbal`、`t_camera2gimbal`
- `yaw_offset`、`pitch_offset`

机器人 YAML 可以增加可选的顶层 `power_rune` 节点，用于覆盖默认算法参数或检测器参数。
其中的相对资源路径以机器人 YAML 所在目录为基准；模块默认配置中的相对路径以默认配置文件
所在目录为基准。

每帧的实时敌方颜色和弹速由调用方传入 `process()`。弹速无效或超出 `10-35 m/s` 时，算法会
使用默认弹速继续计算瞄准角，但强制禁止开火。

## 推理后端

OpenVINO 和 TensorRT 由 CMake 按 CPU 架构自动二选一（互斥，不能同时启用）：
`aarch64`/Jetson 用 TensorRT，`x86_64` 用 OpenVINO。需要强制指定时用
`-DSP_VISION_BACKEND=OPENVINO|TENSORRT|NONE`。

- OpenVINO 直接读取 `models/model-0624.onnx`。
- TensorRT 读取目标设备上生成的 `models/model-0624-fp16.engine`。

TensorRT engine 与 GPU 型号、CUDA 和 TensorRT 版本相关，必须在实际部署的 Jetson 上生成：

```bash
tasks/auto_buff/scripts/build_trt_engine.sh
```

如果 TensorRT 不在系统默认搜索路径，可以显式指定：

```bash
TENSORRT_ROOT=/path/to/TensorRT \
tasks/auto_buff/scripts/build_trt_engine.sh
```

生成的 `*.engine` 已由目录内的 `.gitignore` 排除，不应提交到仓库。

## 构建示例

自动识别（x86_64 走 OpenVINO，aarch64/Jetson 走 TensorRT）：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2
```

强制指定后端（例如在 x86 上编 TensorRT）：

```bash
cmake -S . -B build-tensorrt \
  -DSP_VISION_BACKEND=TENSORRT \
  -DTENSORRT_ROOT=/path/to/TensorRT \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build-tensorrt -j2
```

## 许可证与来源

RP 核心代码基于 RP-26Rune 提交
`7181abb35ffa282356324266427f9cc295f1218b` 适配。五点模型保留了上游发布的 SHA-256 校验。
两者均采用 MIT 许可证，详细信息见：

- `RP26_LICENSE`
- `THIRD_PARTY_NOTICE.md`
- `models/LICENSE`
