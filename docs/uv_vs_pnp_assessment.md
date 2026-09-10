# UV 观测坐标系 vs 当前 PnP + yaw 搜索：评估报告

> 评估对象：`sp_vision_25_rbclone` 当前方案，对比 `rmcs_auto_aim_v2`（南理工）与 `awakening`（中科大/深大系）的 UV 观测方案。
> 结论基于源码逐行比对 + 单帧蒙特卡洛实验 + Fisher 信息量（CRLB）分析，实验代码见 `/tmp/pnp_vs_uv/`。

---

## 0. 一句话结论

**UV 方案更好，但收益主要来自"多装甲/多灯条联合观测 + 噪声按传感器原生空间传播"，而不是"用不用 PnP"本身。**
当前方案的单帧前端（IPPE + 140 步 yaw 搜索）其实已经接近单装甲的 CRLB，所以只换观测空间、仍只喂一块装甲，提升有限；
真正被浪费的是"每帧只用一块装甲"和"手调 R 把深度/朝向信息主动扔掉"。替换难度中等偏低（1.5~3 周），且有一个 1~2 天就能验证的中间步骤。

---

## 1. 两套方案到底在做什么

### 1.1 当前项目：逐帧 PnP → 世界系 4 维观测 → EKF

`tasks/auto_aim/geometry/solver.cpp`：

| 步骤 | 代码 | 说明 |
|---|---|---|
| 单装甲 PnP | `solver.cpp:119` `cv::solvePnP(..., SOLVEPNP_IPPE)` | 每块装甲、每帧解一次，4 个角点 |
| yaw 修正 | `solver.cpp:306-328` `optimize_yaw()` | **不是二次规划**，是 140 个采样点、1° 步长、±70° 的一维网格搜索；代价函数是 4 角点像素距离和（`solver.cpp:364-373`，SJTU_cost 那套权重被注释掉了） |
| 世界系量 | `solver.cpp:133-145` | `xyz_in_gimbal/world`、`ypr_in_gimbal/world`、`ypd_in_world`、`yaw_raw` |

`tasks/auto_aim/tracking/rv_from_fyt.cpp`（11 维 EKF）：

```
状态:  [x, vx, y, vy, z, vz, yaw, vyaw, radius, radius_offset, height_offset]
观测:  [yaw, pitch, distance, armor_yaw]      // 世界/球坐标系
z_  ← armor.ypd_in_world[0..2] + armor.ypr_in_world[0]        // rv_from_fyt.cpp:139-142
h(x) = xyz2ypd(h_armor_xyz(state, id)) + 装甲角                  // rv_from_fyt.cpp:268-277
R    ← 手调常数（rv_from_fyt.cpp:123-142）：
        azimuth/pitch 方差 4e-3  → σ≈3.6°
        distance 方差 log(|Δangle|+1)+1 → σ≈1.0~1.4 m
        armor_yaw 方差 log(d)/200+9e-2  → σ≈18°
```

关键事实：`tracker.cpp:587-601` 的更新循环**匹配到第一块装甲就 `break`**——每帧只有一块装甲进滤波器，其余检测结果全丢。

### 1.2 rmcs_auto_aim_v2：11 维 EKF，观测是灯条端点像素

`src/module/tracker/model/robot.cpp`：

```
状态:  [x,y,z,vx,vy,vz,rotation_angle,rotation_speed,radius_forward,radius_lateral,height_lateral]  (11)
观测:  每条可见灯条 4 维 = [upper.x, upper.y, lower.x, lower.y]（像素）   robot.cpp:449-455
h(x) : 用状态预测整台机器人 → 每条灯条上下端点投影到像素        robot.cpp:159-171
H    : 解析雅可比（含 θ 的数值偏导）                          robot.cpp:234-327
R    : 固定像素噪声 40 px                                    robot.hpp:76
PnP  : 只在初始化用一次                                      robot.cpp:376-379
关联 : 锚点 + 按 x 排序递推 id（所有可见装甲/灯条一起进）      robot.cpp:542-626
```

### 1.3 awakening：13 维 ESKF，观测是灯条 UVL 特征 + ceres Jet

`src/tasks/auto_aim/armor_track/motion_model.hpp`：

```
状态:  [cx,vcx,cy,vcy,cz,vcz,ωz,vyaw,log r1,p1,p2,ωx,ωy]   (13, so3 误差状态)
观测:  每条灯条 4 维 = [atan2(Δx,Δy), center_x, center_y, |Δ|]   (motion_model.hpp:309-317)
h(x) : armor_pose(x,id) → 相机系 → 灯条端点 → 投影(含全畸变)      (motion_model.hpp:275-306)
H    : 误差状态中心差分（update）/ ceres::Jet（predict）          (error_state_extended_kalman_filter.hpp:359-388)
R    : 与灯条像素长度成正比 σ_px = 0.2·len_px（尺度不变）        (armor_target.cpp:371-397)
PnP  : 初始化用 solvePnPGeneric + 正面法线筛选（解镜像歧义）      (armor_target.cpp:171-211)
额外 : 只有 1 块装甲可见时，用 IPPE 的左右灯条深度差做 1 维约束，
        专门治"单装甲深度/半径退化"                          (armor_target.cpp:418-459)
关联 : 可见性预测 + 加权代价贪心 1-1 匹配                         (armor_target.cpp:508-753)
```

---

## 2. 定量对比（实测）

### 2.1 单装甲前端：当前方案其实已经接近最优

蒙特卡洛（fx=1813.6，小装甲 135mm，角点高斯噪声 σ=0.5px，500 次/点；"朝向误差"取装甲法线夹角 mod 180°，因为单装甲法线方向本身有 180° 歧义）：

| 距离 | 表观宽度 | IPPE 位置 RMS | **IPPE 原始朝向 RMS** | **140 步 yaw 搜索** | 6-DOF LM 精修 | yaw CRLB | 深度 CRLB |
|---|---|---|---|---|---|---|---|
| 2 m | 122 px | 0.009 m | 14.0° (p95 31.8°) | 2.07° | 13.98° | 2.11° | 0.008 m |
| 4 m | 61 px | 0.038 m | 20.0° (p95 34.6°) | 4.40° | 20.13° | 4.23° | 0.030 m |
| 6 m | 41 px | 0.097 m | 23.5° (p95 37.9°) | 6.99° | 23.56° | 6.64° | 0.068 m |
| 8 m | 31 px | 0.194 m | 25.9° (p95 41.4°) | 9.08° | 25.94° | 9.03° | 0.122 m |
| 10 m | 25 px | 0.328 m | 25.6° (p95 42.1°) | 11.06° | 25.69° | 11.74° | 0.190 m |

σ=1.0px 时同规律放大：yaw 搜索 4.5°→18.2°，IPPE 原始朝向 18.7°→32.6°。

**读法：**
- IPPE 直接给出的朝向基本不可用（14~26° RMS），因为对称装甲板平面存在镜像解；6-DOF LM 精修也救不回来（同一个局部极小）。
- **当前的 140 步网格搜索反而很好**：它与"位置已知、只估 yaw"的 CRLB 几乎重合（有时还略优，因为 CRLB 是渐近界）。也就是说，作者把位置固定、只搜 yaw 的设计是合理的——比朴素 6 自由度 ML 拟合更有效。
- 位置/深度上 IPPE 也接近 CRLB（8m 时 0.19m vs 界 0.12m）。
- **所以"把 PnP 换成 UV"本身，在单装甲场景下不会带来数量级提升。**

### 2.2 真正的问题：观测信息被扔掉

`rv_from_fyt.cpp` 手调的 R 与实际可达精度对比（6 m，单装甲，σ=0.5px）：

| 观测量 | 当前 R 的 σ | 实测可达精度（CRLB/实验） | 差距 |
|---|---|---|---|
| 方位角 yaw | 3.6° | ≪0.1°（受 IMU/外参误差主导，1° 量级） | 偏保守 |
| 俯仰角 pitch | 3.6° | 同上 | 偏保守 |
| **距离** | **≈1.0~1.4 m** | **0.068 m** | **方差被放大 ~200 倍** |
| **装甲朝向 armor_yaw** | **≈18°** | **6.6°** | **方差被放大 ~7 倍** |

后果：滤波器基本**不信任 PnP 的深度**，深度主要靠"首帧 PnP + 匀速预测 + 模型"维持；目标在纵深方向机动时会出现滞后和漂移，而弹道解算恰恰最吃深度。

### 2.3 多装甲融合：收益的大头（Fisher 信息量）

4 装甲机器人、机器人偏航 45°（2 块装甲可见）、σ=0.5px，状态 `[cx,cy,cz,yaw(,radius)]` 的联合 FIM 边缘 σ：

| 距离 | 可见装甲 | 半径已知 σ_cz | 半径自由 σ_cz | 半径已知 σ_yaw | 半径自由 σ_yaw |
|---|---|---|---|---|---|
| 4 m | 1 | 0.068 m | 0.298 m | 1.15° | 1.74° |
| 4 m | 2 | **0.0075 m** | 0.027 m | **0.41°** | 0.41° |
| 8 m | 1 | 0.178 m | **≈不可观测（10⁵ m）** | 2.54° | 2.54° |
| 8 m | 2 | **0.0275 m** | 0.090 m | **0.87°** | 0.89° |
| 10 m | 1 | 0.247 m | 不可观测 | 2.30° | — |
| 10 m | 2 | **0.044 m** | 0.136 m | **1.12°** | 1.12° |

**读法：**
- 单装甲时，机器人中心深度与半径完全耦合，8m 以上深度**数学上不可观测**（这也解释了作者为什么把距离方差调到 1 m²）。
- 两块装甲可见时，深度 σ 从 0.18 m 降到 0.028 m（**6.5×**），机器人偏航从 2.5° 降到 0.87°（**2.9×**），而且即使半径作为未知状态也稳定可观测。
- **这就是 UV 方案的核心价值**：它能自然地把所有可见灯条写进一个观测方程；当前"每帧一块装甲"的架构做不到。

### 2.4 计算开销

| 环节 | 耗时 |
|---|---|
| `solvePnP(IPPE)` | 12.9 µs |
| **140 步 yaw 网格搜索** | **203.5 µs** |
| `solvePnPRefineLM` | 10.9 µs |
| 一次 `projectPoints`（4 点） | 1.5 µs |

当前每帧前端 ≈ 216 µs（`try_solve` 还可能一帧调两次），其中 94% 花在 yaw 网格搜索上。
UV-EKF 一次多灯条联合更新的 h(x)+H 大约 30~50 µs（解析雅可比）——**换过去反而更快**，NX 上的余量可以留给多装甲/多相机。

---

## 3. UV 坐标系是否更好？

### 明显更好的场景
1. **中远距离 + 多装甲可见**：深度可观测性、旋转相位精度提升 3~6 倍（见 2.3）。
2. **部分遮挡**：UV 可以按灯条单独更新（awakening 的 `match_light`），当前方案一块装甲 4 个角点缺一个就整帧丢弃。
3. **噪声模型**：像素噪声与灯条长度成比例（尺度不变），换长焦/短焦相机时无需重调 R；当前方案每次相机切换要手动膨胀协方差（`rv_from_fyt.cpp:118-121`）。
4. **去掉对 ±70° 搜索窗口的依赖**：当前 yaw 是"逐帧测量"，靠"搜到 gimbal yaw 附近"来定解；
   UV 里 yaw 是状态，由"装甲板固定 15° 倾角"这一先验 + 时序融合自然定解
   （注：平面靶标的镜像解本质上是靠倾角先验破除的，两套方案都用这个先验，
   区别只是 UV 把它放进 h(x) 连续融合，而当前方案是每帧在 ±70° 窗口里离散搜索）。
5. **算力**：见 2.4。

### 提升不明显 / 需谨慎的场景
1. **近距离、只看得见一块装甲**：单装甲 CRLB 决定了下限（8m 深度 σ≥0.12m），UV 不会突破物理。
2. **外参/IMU 姿态误差**：这是两套方案的共同上限。1° 姿态误差在 8m 对应约 31 px 的像素偏移——远大于检测噪声 0.5 px。所以 UV 也必须把像素噪声调大（rmcs 用固定 40 px，awakening 用长度比例）来吸收，**"噪声自动正确传播"的收益会被这一项吃掉一部分**。
3. **模型失配**：UV 的深度依赖机器人半径/几何模型（半径是状态，可以自估，但需要激励）；当前 PnP 深度是**模型无关**的，对英雄大装甲、基地、前哨这类非标准几何更稳。这是当前方案的真实优点，不要丢。
4. **关联错误会直接发散**：UV 多装甲方案必须配套可见性预测 + 门限 + 卡方检验；当前"一块装甲 + 马氏距离选择"虽然笨，但很稳。

### 结论
UV **在架构上更优**（信息不丢、噪声在原生空间、支持多观测、更便宜），但它是"把估计质量从 80 分提到 90 分"的改进，不是"能不能打中"的能力解锁。收益大小高度依赖**每帧能看到几块装甲**：中远距离对枪时经常只有一块可见，这时收益主要来自噪声模型和部分遮挡，而不是多装甲。

---

## 4. 替换难度评估

### 4.1 有利条件（比预期好很多）

| 条件 | 证据 |
|---|---|
| 位姿求解已有接口隔离 | `IArmorPoseSolver`（`model/armor_interfaces.hpp:21-28`），`Tracker::setPoseSolver()` 可换实现 |
| **下游完全不吃 Armor 位姿** | 审计结果：aimer/planner/shooter/commandgener 读 `Armor` 位姿字段 **0 处**；它们只用 `Target` 的 `ekf_x()` / `armor_xyza_list()` / `get_recent_armor_xyzad()`。只要 Target 继续吐世界系 xyz + 装甲 yaw，下游一行不改 |
| 状态量已经是机器人模型 | `rv_from_fyt.cpp:243-255` 的 `h_armor_xyz(state,id)` 已经是"中心 + 半径 + yaw + id·2π/n + offset"的机器人模型，UV 的 h(x) 只是把它再投影到像素 |
| 标定数据齐全且逐帧同步 | `camera_matrix / distort_coeffs / R_camera2gimbal / t_camera2gimbal / R_gimbal2imubody` 全在 yaml（`solver.cpp:36-48`）；`gimbal.q(t-3ms)` 按帧时间戳插值（`rb_auto_aim_debug.cpp:168-170`），无需新增标定 |
| 检测器无需改 | YOLO 输出的 4 关键点顺序是 `[LT, RT, BR, BL]`（`ov_yolos/yolov8.cpp:323-357`），正好就是 2 条灯条的上下端点；传统视觉路径还额外有 `Lightbar.top/bottom` |
| 有离线 A/B 工具 | `tests/auto_aim_test.cpp` + `快/*.avi`+`*.txt`（含四元数，574 MB 实录）+ `Plotter` CSV；已经输出 `armor_x/y/yaw/yaw_raw`、EKF 状态、残差、NIS、aim point、command |
| 已有先例 | `rb_auto_aim_simulator.cpp:624-626,700-702` 已用 `set_camera_calibration`/`set_camera2gimbal`/`set_R_gimbal2world_from_tf` 注入逐帧内外参，说明 UV 所需的输入通路都在 |

### 4.2 需要改的东西（按文件）

| 文件 | 改动 | 规模 |
|---|---|---|
| `tasks/auto_aim/tracking/rv_from_fyt.cpp/.hpp` | 新增 UV 观测模型 `h`、雅可比 `H`、像素噪声 R、初始化（首帧 PnP 复用现有 `Solver`）；或新建 `uv_from_fyt.cpp` 并存 | 主体，300~500 行 |
| `tasks/auto_aim/tracking/target.cpp/.hpp` | 初始化路径（从 `Armor` 位姿改为 PnP 首帧 + 像素观测）；保持公开接口 | 100~200 行 |
| `tasks/auto_aim/tracking/tracker.cpp` | 不再 `break`；多装甲关联/可见性；把角点+相机模型+云台四元数传进滤波器 | 150~300 行 |
| `tasks/auto_aim/geometry/solver.cpp` | 从"每帧主力"降级为"初始化 + 重投影调试 + 前哨 pitch 判别"；`optimize_yaw` 可保留但不再每帧调用 | 少量 |
| `configs/*.yaml` | 新增 UV 噪声参数（σ_px 比例、σ_angle、σ_length）、可见性阈值、关联门限 | 小 |
| `tasks/auto_aim/detection/*` | 不需要改（角点已够） | 0 |

### 4.3 分阶段路线与工作量

| 阶段 | 内容 | 风险 | 工时 |
|---|---|---|---|
| **0. 基线** | 用 `tests/auto_aim_test.cpp` 跑 `快/` 全部录像，记录 aim point 抖动、深度轨迹、收敛时间、丢帧率、NIS | 无 | 0.5~1 天 |
| **1. 多装甲 ypd 融合（不换观测空间）** | `tracker.cpp` 去掉 `break`，对每块匹配装甲各做一次现有 4 维更新 | 低 | 1~2 天 |
| **2. 单装甲 UV 观测** | 用同一块装甲的 4 角点做像素观测，替换 `h/h_jacobian/R`，PnP 只用于初始化 | 中 | 3~5 天 |
| **3. 多装甲 UV 联合更新** | 可见性模型 + id 关联 + 批量更新 + 单装甲退化保护（可借用 IPPE 深度差约束） | 中高 | 4~6 天 |
| **4. 特殊目标与回归** | 前哨（3 装甲 + 高度偏移）、平衡（2 装甲）、基地、长焦/短焦切换、全 `src/*.cpp` 回归 | 中 | 2~4 天 |

**合计约 2~3 周**（单人、熟悉代码库）。如果只想验证收益，**阶段 0+1 用 2~3 天就能给出结论**。

### 4.4 主要风险

1. **Q/R 重调**：UV 方案的性能几乎全在噪声参数上；建议直接抄 awakening 的尺度不变模型（σ_px=0.2·len_px, σ_len=0.5·len_px, σ_angle=0.5 rad）作为起点，再按实录微调。
2. **关联错误 → 发散**：必须加卡方门限 + `diverged()` 保护 + 旧方案回退开关。
3. **模型失配 → 深度偏**：半径是状态可自估，但前哨/英雄/基地要单独建模（当前 `h_armor_xyz` 已有 outpost 高度乘数与 4 装甲半径偏移，可复用）。
4. **姿态误差上限**：无论哪种方案，IMU/外参 1° 误差在 8m 就是 0.14 m；建议顺手评估把相机-云台外参与 IMU 零偏的残差建模进去（或至少把 UV 噪声按距离/姿态误差自适应）。

---

## 5. 建议

1. **不要为了"上 UV"而上 UV。** 先做阶段 0+1（多装甲融合，2~3 天），如果离线 A/B 显示深度轨迹和 aim point 抖动明显改善，就说明瓶颈确实是"信息集合"，再投入阶段 2~4。
2. 阶段 2 起，**保留现有 tracker 作为可选后端**（配置开关），用同一批录像做 A/B（同一份 `Plotter` 字段，直接比 CSV）。
3. 复用而不是重写：`Solver` 保留做首帧初始化（两个参考项目也都只在初始化用 PnP）、做重投影调试、做前哨 pitch 判别；`reproject_armor` 直接可以当 UV 的 h(x) 的验证工具。
4. 优先移植 awakening 的两个工程细节，而不是它的 ceres 依赖：
   - **长度比例像素噪声**（尺度不变，换相机不用重调）；
   - **单装甲退化时用 IPPE 左右灯条深度差做 1 维约束**（治深度/半径耦合，正好是 2.3 表里"半径自由 → 深度不可观测"那个问题）。
5. 关联逻辑优先参考 rmcs（更简单：锚点 + 按 x 排序递推 id），awakening 的贪心匹配更完备但代码量大。

---

## 附：实验复现

```bash
# 单装甲前端 vs CRLB（500 次蒙特卡洛）
cd /tmp/pnp_vs_uv && g++ -O2 -fopenmp -I/usr/include/eigen3 -I/usr/include/opencv4 \
    bench2.cpp -o bench2 -lopencv_core -lopencv_calib3d && ./bench2 500 0.5

# 多装甲联合 FIM（半径已知/自由）
g++ -O2 -I/usr/include/eigen3 -I/usr/include/opencv4 multi2.cpp -o multi2 \
    -lopencv_core -lopencv_calib3d && ./multi2

# 前端耗时
g++ -O2 -I/usr/include/eigen3 -I/usr/include/opencv4 timing.cpp -o timing \
    -lopencv_core -lopencv_calib3d && ./timing
```

> 注意：本机系统 OpenCV 4.5.4 的 `projectPoints` 不接受 `vector<Point3d>` 输入配 `vector<Point2f>` 输出，必须用 `Point3f`（项目里 `BIG/SMALL_ARMOR_POINTS` 正好是 `Point3f`，不受影响）。
