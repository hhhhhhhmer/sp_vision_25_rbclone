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

- `RVfromFYT::kf_predict()`：按整车平移、旋转和半径模型预测 11 维状态。
- `CenterAccelerationEstimator`：对最近的 EKF 后验整车中心 XY 位置做二次拟合，结果只作为下一跟踪帧的短时预测输入。
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
- `predict(state, armor_id, type)` / `jacobian(state, armor_id, type)`：由 EKF 状态投影出观测及其 8x11 雅可比。
- `noise(state, armor_id, type, z)`：中心像素噪声按姿态误差给大值，长度噪声按端点检测噪声给小值，各向异性由参数天然表达；`attitude_common_ratio` 控制姿态误差里"整帧共模"与"各灯条独立"的比例。
- `associate_gate_px`、`facing_cos_min`：灯条中心像素距离的关联门限，以及装甲板法线与视线夹角余弦的可见性判据。
- `radius_prior_sigma`、`process_noise_z_scale`：单装甲退化时的半径先验，以及深度方向的过程噪声缩放。

UV 观测的好处：长度是差分量，对云台姿态误差（共模平移）不敏感，是深度信息的主要来源；装甲板 4 角点是对称矩形，
法线翻转 180° 时投影点集合不变，因此天生没有 PnP 的镜像解歧义。

`Tracker` 侧的相关接口：`uv_enabled()` / `set_uv_enabled()`、`last_update_count()`（本帧实际参与融合的观测数）；
配置键为 `uv_observation`、`uv_sigma_px`、`uv_sigma_len_ratio`、`uv_sigma_attitude_deg`、
`uv_attitude_common_ratio`、`uv_sigma_angle`、`uv_associate_gate_px`、`uv_facing_cos_min`、
`uv_radius_prior_sigma`、`uv_process_noise_z_scale`（见 `configs/README.md`）。
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
