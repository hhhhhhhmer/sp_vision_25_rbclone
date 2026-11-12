# Tracking

## 作用

把连续帧中的装甲板关联成同一个目标，并维护目标的位置、速度、旋转半径、yaw 和丢失状态。

```text
std::list<Armor> + timestamp -> Tracker -> std::list<Target>
```

## 主要类和函数

### `Tracker`

- `track(armors, t, cam_is_short = true, use_enemy_color = true)`：普通自瞄入口；过滤颜色、关联目标并推进状态机。传统 ypd 观测下逐帧调用 `Solver::try_solve()`；UV 观测（`uv_observation: true`）下 PnP 只在建立新目标时执行一次。
- `sb_track(armors, t)`：哨兵使用的跟踪入口，包含相应目标选择策略；`enemy_color: "auto"` 时按哨兵策略取色。
- `track(detection_queue, armors, t)`：融合全向感知结果与自瞄装甲板。
- `reset()`：把状态机强制置回 `lost`，并清零检测/丢失计数与中心加速度拟合历史（返回 `void`；`target_` 要等下一帧 `set_target()` 才被覆盖）。
- `state()`：返回当前状态名称，便于日志和调试。
- `setSolver()` / `setPoseSolver()`：注入位姿求解接口，测试时可以替换成假实现。
- `set_gimbal()`、`set_fft()`：提供云台状态和周期运动分析器。
- `test_track()`：测试用入口，可直接注入观测；`get_update_count()` / `last_update_count()` 分别返回目标累计更新次数与上一帧实际融合的观测数。

内部关键函数：`set_target()` 根据第一帧和目标类型初始化 EKF；`update_target()` 先预测，再选择同名称、同尺寸的装甲板校正；`state_machine()` 管理状态变化。

### `Target`

- `Target(armor, t, radius, armor_num, P0)`：用第一块装甲板初始化目标和 EKF。
- `predict(t)` / `predict(dt)`：按运动模型把状态预测到指定时刻。
- `update(armor)`：用新的装甲板观测校正 EKF。
- `ekf_x()`：读取当前状态向量。
- `armor_xyza_list()`：根据整车中心、半径和 yaw 还原所有装甲板的 `[x,y,z,yaw]`。
- `diverged()`、`convergened()`：判断滤波器是否发散或已经稳定。

`Target` 是跟踪模块对外提供的目标对象；它负责目标语义和生命周期，把具体矩阵计算交给状态估计器。

### 状态估计器

- `RVfromFYT::kf_predict()`：按整车平移、旋转和半径模型预测 17 维状态：`[x,vx,y,vy,z,vz,yaw,vyaw,r, lat_off0..3, h_off0..3]`。四装甲普通目标每块板使用独立的左右 + 高度偏置；三/两装甲只用基础半径/高度，前哨站把 `[13]` 当高度阶梯。
- 每板偏置用**零均值相对参数化**：只估计前 3 块的左右/高度，第 4 块由负和导出，公共模式由 yaw/半径/中心吸收；P0=σ²=0.05² 允许从 0 在线学习任意非对称布局，对称布局会收敛到 0。YAML 的 `armor_lateral_offsets` / `armor_height_offsets` 是可选先验（顺序前/左/后/右，切向 + 为自身左侧），不填/全 0 也能收敛。`select_aim_armor()` / `aim_armor_xyzad()` 使用选中板自己的偏置。
- `CenterAccelerationEstimator`：对最近的 EKF 后验整车中心 XY 位置做二次拟合，结果只作为下一跟踪帧的短时预测输入；**保留开关、默认关闭**，需在 YAML 里显式设置 `center_accel_ff_enabled: true`。当前实现（0.25 s 拟合 + EMA）在平移靶上加速段有效、脉冲结束后过冲，净亏；修好估计器前不要默认开启。
- `prepare_measurement()`：把装甲板位置转换成 `[yaw,pitch,distance,armor_yaw]` 观测。
- `select_armor_id()`：用马氏距离选择观测对应的装甲板编号。
- `correct()`：使用选定装甲板执行 EKF 校正。
- `armor_xyza_list()`：从整车状态还原所有装甲板位置。
- `CAFromTJU::predict_model()`：执行常加速度状态预测。
- `State2Est`：两种状态估计器的公共接口。

`RVfromFYT` 是当前 `Target` 唯一在用的估计器（`target.hpp` 里硬编码 `RVfromFYT ekf_; State2Est* est = &ekf_;`，没有切换路径）；
`CAFromTJU` 是不显式建模装甲板半径的常加速度模型，目前只被单元测试引用。

### UV（像素/UVL）观测模型

默认的观测是整车坐标系下的 `[yaw,pitch,distance,armor_yaw]`；开启 UV 后改用灯条的图像特征做校正。

- `UvModel::observation(points)`：把装甲板 4 个角点转成 8 维 UVL 观测——
  左/右灯条各 `(角度, 中心x, 中心y, 长度)`；角度用 `atan2(dx,-dy)`，竖直灯条为 0，绕开了 `atan2(dx,dy)` 在 ±π 处的回绕。
- `predict(state, armor_id, type)` / `jacobian(state, armor_id, type)`：由 EKF 状态投影出观测及其 8x17 雅可比。
- `noise(state, armor_id, type, z)`：中心像素噪声按姿态误差给大值，长度噪声按端点检测噪声给小值，各向异性由参数天然表达；`attitude_common_ratio` 控制姿态误差里"整帧共模"与"各灯条独立"的比例。
- `associate_gate_px`、`facing_cos_min`：灯条中心像素距离的关联门限，以及装甲板法线与视线夹角余弦的可见性判据。
- `radius_prior_sigma`、`process_noise_z_scale`：单装甲退化时的半径先验，以及深度方向的过程噪声缩放
  （`Q_z = v1·scale`；深度只由灯条长度反推，闭环自瞄下要平滑就调小）。

UV 观测的好处：长度是差分量，对云台姿态误差（共模平移）不敏感，是深度信息的主要来源；装甲板 4 角点是对称矩形，
法线翻转 180° 时投影点集合不变，因此天生没有 PnP 的镜像解歧义。

**代价是装甲朝向 θ 变弱了**：UV 下 θ 只有"两条灯条间距（宽度）"这一路信息，而宽度对 θ 的导数正比于
`sin(相对角)`，正对相机时恰好为 0（那一刻正是云台瞄着它的时候）。闭环自瞄自转靶时，θ 的相位滞后会经
`r/d` 杠杆（`r=0.2m`、4m 处约 2.9°）变成云台自激。为此加了 `pnp_yaw_*` 一族配置：

- `Tracker::attach_pnp_yaw()`：UV 模式下每帧对候选装甲板调用一次 `Solver::try_solve_yaw()`
  （一次 `solvePnP` + 以云台朝向为中心 ±20°/20 步的窄窗搜索，约 30~90us），把结果写进
  `Armor::ypr_in_world[0]` 并置 `Armor::pnp_yaw_valid`。窗口窄所以不会像 ±70° 全窗口那样跳到镜像解/相邻板。
- `RVfromFYT::add_uv_observation()` 把它挂到对应装甲板的待更新观测上，`correct_uv_batch()` 在 UV 联合更新
  **之后**再做一次 1 维更新：`z=[θ_measured]`、`H` 只有第 6 列，σ 默认 5°。带 3σ 硬门限，
  正对时 PnP 在 ±φ 之间跳产生的野值会被拒绝（否则会污染 NIS 统计、触发 "Bad Converge Found!" 丢跟踪）。
- 效果上等价于"UV 给位置/深度，PnP 给 θ"；`uv_pnp_yaw: false` 可退回纯 UV 做 A/B。

`Tracker` 侧的相关接口：`uv_enabled()` / `set_uv_enabled()`、`last_update_count()`（本帧实际参与融合的观测数）；
配置键为 `uv_observation`、`uv_sigma_px`、`uv_sigma_len_ratio`、`uv_sigma_attitude_deg`、
`uv_attitude_common_ratio`、`uv_sigma_angle`、`uv_associate_gate_px`、`uv_facing_cos_min`、
`uv_radius_prior_sigma`、`uv_process_noise_z_scale`、`uv_pnp_yaw`、`uv_pnp_yaw_sigma_deg`、
`uv_pnp_yaw_window_deg`、`uv_pnp_yaw_steps`（见 `configs/README.md`）。
可视化由 `tools/uv_overlay.hpp` 提供。

### 其他辅助

- `Voter::vote()` / `count()`：累计颜色、编号和装甲板类型出现次数。当前跟踪链路并未使用，属保留的工具类。
- `enemy_color_from_gimbal()`：把下位机颜色编码转换成视觉颜色。
- `filter_enemy_armors()`：从候选列表中删除非敌方颜色装甲板。

### 多装甲板融合

- `multi_armor_fusion`：是否把同一帧里多块匹配装甲板都送进 EKF（默认 `false`；前哨站和基地始终只更新单板）。
- `multi_armor_gate`：融合第二块及之后装甲板的马氏距离门限。
- `multi_armor_angle_sigma_scale`：融合第二块及之后装甲板时，其 PnP 朝向的噪声放大系数。
- `meas_azimuth_sigma` / `meas_distance_sigma` / `meas_angle_sigma`：观测噪声标准差的覆盖值，`<= 0` 时沿用 EKF 内部的启发式。
- `Target::update(armor, max_mahalanobis)`、`update(armor, max_mahalanobis, exclude_ids, angle_sigma_scale)`、`update_batch(armors, gate, max_armors)` 与 `last_batch_observation_count()` 是融合的底层入口。

## 原理

跟踪主链路状态为 `lost -> detecting -> tracking -> temp_lost`；融合全向感知时还会进入 `switching`（等待主相机出现同优先级目标）。新目标需要连续检测若干帧才进入稳定跟踪；短暂丢失时继续依靠运动模型预测，超过阈值后才彻底丢弃。匹配成功后，EKF 用新观测校正预测状态。

不同目标使用不同装甲板数量、初始半径和协方差，例如平衡步兵按两块装甲板建模，前哨站和基地按三块建模，普通车辆按四块建模。

EKF 的预测阶段利用运动模型得到下一时刻状态和不确定度；校正阶段将实际观测与预测观测作差，再按协方差权重修正状态。角度残差必须归一化，否则跨越 `-pi/pi` 时会产生错误跳变。

普通车辆的帧间预测可使用上一帧确认的中心 XY 加速度，形式为 `F*x+B*a`。加速度估计器由 `Tracker` 持有，不写入 `Target`，因此 Planner、弹道补偿和旧 Aimer 对目标副本的未来外推仍保持 CV；现有 Z 轴周期 `Wave` 补偿不受影响。

修改丢失和重新锁定逻辑从 `tracker.cpp` 开始；修改目标生命周期从 `target.cpp` 开始；修改状态方程和观测方程从 `rv_from_fyt.cpp` 或 `ca_from_tju.cpp` 开始。
