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
| `rp_core/include/RuneUvPoseFilter.hpp`、`rp_core/src/RuneUvPoseFilter.cpp` | 符平面 UV 观测 EKF（逐帧替代 PnP） |
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

## UV 观测模式（默认开启）

符的自瞄与装甲板走同一套思路：**逐帧不再调用 PnP**，而是把网络五点和轮廓锚框点直接
作为像素观测，用一个 7 维 EKF 维护符平面位姿：

- 状态：`[旋转(3), 平移(3), 绕符平面法向的角速度(1)]`，旋转按右乘误差更新；
  状态在 `world_to_rp` 后的固定 RP 系里（代码沿用了 `car` 的命名），相机/云台转动
  只影响像素观测，不参与状态传播；
- 观测：每片未激活符叶的 5 个网络关键点（外侧、左侧、中心 R、右侧、内侧）+
  4 个整片符叶 PCA 锚框点（外左、内左、内右、外右）；
- 预测：只按角速度让符叶绕自身 z 轴旋转；角速度通过状态转移协方差与旋转耦合，
  由 UV 观测间接更新；
- 目标切换：只有 1 号位（当前跟踪片）允许尝试 72° 整数倍旋转假设，明显更优时
  重定义状态帧；其他位置号只做门限/拒绝，避免 5 次对称性把位置号误差吸收成
  状态旋转、污染 1 号位；
- 兜底：本帧没有任何可接受的 UV 观测时，立即回退到原有 PnP + Chamfer 重建，
  因此短时遮挡/快速运动不会把过期位姿送进火控。PnP 兜底重建会保留已经收敛的
  角速度（首次建立目标和模式切换仍从 0 开始），避免每掉一帧就重新收敛。

相关配置都在 `project` 节点下，机器人 YAML 可用顶层 `power_rune.project` 覆盖：

| 字段 | 默认值 | 作用 |
| --- | --- | --- |
| `uv_observation` | `true` | 是否启用 UV 观测；`false` 时完全走原 PnP 链路 |
| `uv_sigma_px` | `3.0` | 网络关键点检测噪声，px |
| `uv_sigma_center_px` | `1.5` | 中心 R 点检测噪声，px |
| `uv_anchor_sigma_px` | `4.0` | 轮廓锚框点检测噪声，px |
| `uv_gate_chi2` | `200.0` | 单符叶观测卡方门限 |
| `uv_switch_rmse_px` | `10.0` | 触发 72° 目标切换搜索的预测 RMSE 阈值，px |
| `uv_initial_rotation_sigma_deg` | `5.0` | PnP 初值的旋转标准差，deg |
| `uv_initial_translation_sigma_m` | `0.15` | PnP 初值的平移标准差，m |
| `uv_initial_angular_velocity_sigma` | `2.0` | 角速度初值标准差，rad/s |
| `uv_process_rotation_sigma_deg` | `0.2` | 每帧旋转过程噪声，deg |
| `uv_process_translation_sigma_m` | `0.015` | 每帧平移过程噪声，m |
| `uv_process_angular_velocity_sigma` | `1.0` | 角速度过程噪声，rad/s |
| `uv_max_prediction_gap_s` | `0.25` | 超过该间隔不信任预测，直接重建，s |
| `uv_circle_radius` | `0.15` | 靶心圆半径，m |
| `uv_circle_center_radius` | `0.7` | 靶心圆中心到符心距离，m |

调试接口：`RuneDebugSnapshot::uv_pose_active`、`uv_reprojection_error`、`uv_nis`、
`uv_rejected_blades`。其中 `uv_pose_active` 只在本帧 UV 更新成功时为 true，PnP
初值/兜底帧为 false；`rune_debug_draw.hpp` 会在画面上显示 UV 状态和重投影 RMSE。

### 视频回放测试

`tests/auto_buff_test.cpp` 支持指定起始帧、帧数和命令 CSV：

```bash
./build/auto_buff_test --headless=true --config-path=configs/xiaohei.yaml \
  --start-frame=14400 --max-frames=300 --mode=small --color=red \
  --dump-commands=/tmp/red_uv.csv yolo_buff/123/红.mp4
```

在 `yolo_buff/123` 的蓝符、大符和红符片段上，UV 模式与 PnP 的 `found_frames` 基本一致
（300 帧内相差 0~2 帧），大部分帧使用 UV 跟踪；遮挡/快速运动导致的少数帧会自动回退到 PnP。
注意离线视频没有 IMU，测试程序传入的是单位四元数；UV 状态在固定 RP 系中，所以
云台转动本来就不应该改变状态，实车 IMU 只用于把观测投到像素，不会额外引入预测旋转。

`auto_buff_test` 会同时输出一个 PnP/UV 共用的精度指标 `kp_rmse_avg`：用最终位姿把
1 号位未激活符叶的 5 个网络关键点重投影回图像，统计像素 RMSE。当前机器上的实测
（300 帧，`-O3 Release`）：

| 视频片段 | 模式 | found | 1 号位五点 RMSE | PnP 帧数 | core 平均耗时 |
| --- | --- | ---: | ---: | ---: | ---: |
| `蓝.mp4`（small/blue） | PnP | 253 | 7.84 px | 279 | 1.45 ms |
| `蓝.mp4`（small/blue） | UV | 253 | **3.15 px** | **15** | **1.03 ms** |
| `big_buff.avi`（big/blue） | PnP | 158 | 9.62 px | 257 | 1.79 ms |
| `big_buff.avi`（big/blue） | UV | 158 | **5.14 px** | **35** | **1.34 ms** |
| `红.mp4`（第 14400 帧起，small/red） | PnP | 201 | 6.45 px | 227 | 1.18 ms |
| `红.mp4`（第 14400 帧起，small/red） | UV | 201 | **2.36 px** | **25** | **0.83 ms** |

折算下来：

- 1 号位关键点重投影 RMSE：小符降低约 **60%**，大符降低约 **46%**；
- 逐帧 PnP/Chamfer 次数：降低约 **86%~95%**（只剩建立目标、兜底帧）；
- `core_ms`：降低约 **25%~30%**；
- `found_frames`：三段视频与 PnP 完全一致；
- 命令稳定性：UV 与 PnP 的 yaw/pitch 差异 RMS 在 0.02° 以内（蓝符约 0.004°/0.006°，
  红符约 0.015°/0.018°），开火决策在共同 found 帧上约 97% 一致；二阶差分 RMS 两者
  接近，蓝符 UV 略低、红符 pitch 略高，具体数值受视频片段影响较大，建议以实车数据为准。

注：UV 模式的 `kp_rmse_avg` 只统计 UV 更新成功的帧，PnP 初值/兜底帧没有计入；
PnP 基线包含全部帧。因此 UV 的 RMSE 数值略偏乐观，比较时请留意统计口径。

这些数字是离线视频、单位 IMU 下的结果；UV 状态在固定系、预测只含符自身角速度，
所以实车真实 IMU 主要改善观测投影和云台运动下的 PnP 初值，不会改变这里的预测模型。

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
