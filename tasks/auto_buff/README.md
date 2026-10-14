# RP-26Rune 全链路接入说明

`auto_buff` 负责完整的能量机关处理链路，包括五点模型推理、语义轮廓精修、Chamfer 位姿
优化、大小符运动估计、弹道拦截和自动火控。相机取帧、IMU 姿态插值、云台通信及机器人
标定仍由当前项目原有模块负责。

## 目录结构

| 路径 | 内容 |
| --- | --- |
| `rune_system.hpp/.cpp` | 对外入口 `auto_buff::RuneSystem`：`process()` / `reset()` / `debug_snapshot()`；以及 `BuffMode`、`EnemyColor`、`RuneFailureReason`、`RuneCommand`、`RuneDebugSnapshot` |
| `detector/` | `RuneDetector` 五点推理（OpenVINO / TensorRT 两套后端）及 TRT 预处理 kernel |
| `adapter/pose_adapter.*` | 项目 IMU 姿态 → RP 核心 `CameraPose` 适配 |
| `rp_core/include`、`rp_core/src` | 上游 RP 算法：`PowerRuneProcessor`、`RuneDecisionModule`、`PowerRunePlane`（Chamfer 位姿）、`RuneObservationRefiner`、`PhaseMotionEstimator` 与 `big_rune_motion_estimate/`、`PowerRuneBallisticModel`；配置中心 `PowerRuneConfigStore`（全局 `J_POWER_RUNE`） |
| `rp_core/config/` | `json.hpp`（YAML 包装）、`PnPVariable.hpp` |
| `config/power_rune_defaults.yaml` | RP 算法默认参数 |
| `models/` | `model-0624.onnx`（+ 本机生成的 `*-fp16.engine`） |
| `rune_debug_draw.hpp` | 基于 `RuneDebugSnapshot` 的 OpenCV 调试绘制 |
| `scripts/build_trt_engine.sh` | TensorRT engine 构建（构建前校验模型 SHA-256） |

`auto_buff` 编译为静态库，只在 `SP_VISION_BACKEND` 为 `OPENVINO`/`TENSORRT` 时加入构建
（`NONE` 时整个模块不编译）。可直接参考的调用与测试：
`tests/sp_example/rune_system_example.hpp`、`tests/auto_buff_test.cpp`、
`tests/rune_system_safety_test.cpp`、`tests/rune_config_test.cpp`、
`tests/rune_motion_fit_test.cpp`、`tests/rune_pose_adapter_test.cpp`、
`tests/rune_detector_decode_test.cpp`。

## 公共接口

调用方只需包含 `tasks/auto_buff/rune_system.hpp`，使用机器人 YAML 配置构造一个
`auto_buff::RuneSystem`，然后在固定的图像线程中调用 `process()`。退出打符模式时必须调用
`reset()`，清除运动拟合、目标选择和开火冷却状态。

返回的 yaw、pitch 遵循当前项目的坐标约定，单位均为弧度。第一版不提供速度和加速度前馈，
调用 `gimbal.send()` 时对应字段保持为零。

`debug_snapshot()` 提供五点检测结果、语义轮廓、模型重投影、预测命中点、运动参数、阶段耗时
和失败关闭原因。算法核心不会创建或管理 GUI 窗口，调试渲染由调用方完成：参考实现见
`rune_debug_draw.hpp`（OpenCV 绘制）和 `tests/auto_buff_test.cpp`；`rp_core` 里
`foxglove::schemas` 下的结构体只是本地兼容桩，不产生实际可视化输出。

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

脚本默认把 engine 写到 `models/model-0624-fp16.engine`，也可以传入输出路径，或用 `TRTEXEC`
指定 `trtexec`：

```bash
tasks/auto_buff/scripts/build_trt_engine.sh /path/to/model-0624-fp16.engine
TRTEXEC=/path/to/trtexec tasks/auto_buff/scripts/build_trt_engine.sh
```

构建前脚本会校验 `model-0624.onnx` 的 SHA-256，不匹配时直接退出。

生成的 `*.engine` 已由 `models/.gitignore`（`*.engine`）排除，不应提交到仓库。

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
`7181abb35ffa282356324266427f9cc295f1218b` 适配。五点模型的 SHA-256 在
`scripts/build_trt_engine.sh` 中构建 TensorRT engine 前校验，OpenVINO 运行时不做校验。

上游 RP-26Rune 为 MIT 许可证，但本仓库目前**没有附带 RP 核心的许可证文本**
（`rp_core/` 内也没有许可证头），重新分发前需要补一份上游 MIT 许可证全文。
五点模型的来源声明与 MIT 许可证文本见：

- `THIRD_PARTY_NOTICE.md`
- `models/LICENSE`
