# Planner

## 作用

把目标未来运动转换成一段期望云台轨迹，再由 MPC 生成当前时刻需要发送的角度、速度和加速度。

## 主要类型和函数

### 输出和入口

- `Plan`：包含 `control`、`fire`、目标 yaw/pitch（`target_yaw/target_pitch`，理想瞄点，供日志与门限用）、控制 yaw/pitch、速度、加速度。
- `Planner::plan(optional<Target>, bullet_speed, gimbal_yaw, strategy)`：统一入口；先补偿目标模型延迟，再分派具体策略。
- `plan(Target, bullet_speed)`：通用动力学策略（不需要云台反馈）。
- `rbplan()`、`sbplan()`、`rbHeroplan()`：步兵压制 / 哨兵 / 英雄策略，三者都只吃**一个** `gimbal_yaw`。
- `rbShoot()`：根据装甲板位置、角度误差和特殊目标条件判断是否开火。

### 云台反馈只有一个入口：`gimbal_yaw`

- 单位是**度**（与 `io::GimbalState::yaw` 的约定一致）。俯仰角与角速度**不再需要**。
- 它**只用于开火判据**：
  - `rbplan` → `is_fire(gimbal_yaw/57.3 − yaw_offset_, ...)`，与装甲板几何窗比较；
  - `sbplan` → 先过 `|gimbal_yaw/57.3 − yaw_offset_ − plan.target_yaw| < 3°` 粗门，再叠 MPC 残差门。
- 它**不进 MPC**：MPC 初值固定取参考轨迹第 0 拍（`x0 << traj(0,0), traj(1,0)`），
  所以下发的命令完全由目标决定、不随云台反馈抖动，也不受反馈噪声影响。
- ⚠️ 由此产生的后果：MPC 残差判据 `hypot(traj(k_f) − x(k_f)) < fire_thresh_` 的两边现在都在参考轨迹上，
  残差恒 ≈0，**实际不构成判据**。`SB` 策略真正把关的是那 3° 粗门；`Dynamics` 策略等于没有开火判据。
  若将来需要"追不追得上"的预判，必须先把云台真实状态喂回 `x0`，否则那条门永远是 0。

### 瞄准与轨迹

- `aim()`、`rbaim()`、`heroaim()`：选择装甲板并结合弹道计算期望 yaw/pitch。
- `get_trajectory()`、`rbget_trajectory()`：在规划时间域内不断预测目标，生成 `[yaw, yaw_vel, pitch, pitch_vel]` 参考轨迹。
- `setup_yaw_solver()`、`setup_pitch_solver()`：分别建立两轴 TinyMPC，并设置加速度边界与 Q/R 权重。

## 原理

规划时间步长为 `DT=0.01 s`，共 `HORIZON=100` 个点。首先用 `Target::predict()` 和弹道飞行时间得到每个未来时刻的瞄准角，再用中心差分得到角速度，组成参考轨迹。

yaw 和 pitch 分别使用二阶离散模型：状态是角度和角速度，输入是角加速度。MPC 在跟踪误差、控制量大小和最大加速度约束之间求折中。MPC 的初始状态取**参考轨迹第 0 拍**（见上节），程序取时间域中点作为当前应发送的命令。

开火判据由**两道真实量**把关：`is_fire()` 的装甲角宽窗（`rbplan`）与 3° 粗门（`sbplan`），二者都用真实云台偏航角；MPC 残差门当前不生效（原因见上节）。`rbplan()` 还会根据命令速度查询 yaw 延迟曲线，并在换向时加入额外惩罚。

## 阅读顺序

先看 `planner.hpp` 中的 `Plan` 和策略枚举，再看 `planner.cpp` 的统一入口，然后看 `planner_trajectory.cpp`。只有需要调整 Q/R、约束或求解器时才进入 `planner_mpc.cpp` 和 `tinympc/`。
