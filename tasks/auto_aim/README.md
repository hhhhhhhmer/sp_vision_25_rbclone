# Auto Aim

## 整体流程

自瞄代码按数据流拆成五个阶段：

```text
Camera::read()
    -> YOLO::detect() / Detector::detect()
    -> Solver::try_solve()      // 仅传统 ypd 观测；UV 观测（uv_observation: true）下只在建立新目标时解一次
    -> Tracker::track()
    -> Planner::plan() / Aimer::aim()
    -> Gimbal::send()
```

入口程序仍然位于 `src/`。本目录只实现算法模块，不负责决定程序运行哪一种模式。

## 子目录

| 目录 | 主要类或函数 | 作用 |
| --- | --- | --- |
| `model/` | `Armor`、`IArmorDetector`、`IArmorPoseSolver`、`CameraGeometry` | 保存单帧观测数据、公共接口，以及 UV 观测所需的相机几何 |
| `detection/` | `YOLO::detect()`、`Detector::detect()`、`Classifier`、`rv_aim::Detector`、`MultiThreadDetector` | 从图像中找出装甲板及其类别、关键点 |
| `geometry/` | `Solver::try_solve()`、`Solver::camera_geometry()` | 把二维关键点转换为三维位置和姿态 |
| `tracking/` | `Tracker::track()`、`Target::predict()`、`RVfromFYT`、`UvModel`（UV 观测）、多装甲板融合 | 跨帧关联观测并估计目标运动状态 |
| `aiming/` | `Planner::plan()`、`Aimer::aim()`、`Shooter::shoot()` | 预测击打位置并生成云台和开火命令 |

## 基本原理

识别模块只回答“图像里看到了什么”；几何模块回答“目标在三维空间哪里”；跟踪模块利用连续观测回答“目标正在怎样运动”；瞄准模块再考虑弹丸飞行时间、系统延迟和云台约束，计算控制量。各阶段通过 `Armor`、`Target`、`Plan` 或 `io::Command` 传递结果。

## 推荐阅读顺序

1. `model/armor.hpp`：先认识一帧检测结果。
2. `detection/yolo.hpp`：了解统一识别入口。
3. `geometry/solver.hpp`：了解坐标变换。
4. `tracking/tracker.hpp` 和 `tracking/target.hpp`：了解目标状态。
5. `aiming/planner/planner.hpp`：了解最终控制输出。

本目录由上层 `CMakeLists.txt` 创建 `auto_aim` 目标，不需要单独运行 CMake。
