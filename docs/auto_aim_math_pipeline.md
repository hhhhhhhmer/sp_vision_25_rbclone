# auto_aim 全链路数学过程：从一帧观测到下行 (θ, ω, α)

本文只讲**数学**：每一步的输入是什么量、维度多少、用什么公式变成下一步的量、时间戳怎么对齐、单位和符号怎么定。
入口以步兵标准链路为准：`src/rb_auto_standard.cpp`（`ShootStrategy::rbSuppressiveFire`）。

```text
图像 4 角点(px)
  └─ Solver::try_solve     solvePnP(IPPE) + 140 步 yaw 重投影搜索
       └─ Armor{ xyz_world, ypr_world, ypd_world }          (3+3+3 维)
            └─ Target::update / update_batch                EKF 校正(11 维状态)
                 └─ armor_xyza_list()                       4 块装甲板 {x,y,z,yaw}
                      └─ Planner::rbplan                    延迟补偿 → 弹道 → 参考轨迹(100 拍)
                           └─ TinyMPC 解 QP                 x(2×100), u(1×99)
                                └─ write_mpc_commands       取第 50 拍
                                     └─ Plan{yaw,yaw_vel,yaw_acc,pitch,pitch_vel,pitch_acc,fire}
                                          └─ Gimbal::send   VisionToGimbal 29 字节 @460800
```

---

## 0. 记号、坐标系、单位

| 记号 | 含义 |
| --- | --- |
| **W** | 世界系。原点 = 云台旋转中心（IMU 中心），z 轴竖直向上，x/y 由云台姿态确定 |
| **G** | 云台系（`R_gimbal2imubody` 定义的机械系） |
| **C** | 相机系（光心为原点，z 向前） |
| **A** | 装甲板系。x 垂直板面**朝外**，y 沿板宽，z 沿灯条长 |
| `R_gw` | `R_gimbal2world`，云台→世界旋转 |
| `R_cg, t_cg` | 相机→云台的外参（`R_camera2gimbal`, `t_camera2gimbal`） |
| `q` | 下位机上行 IMU 四元数 (w,x,y,z) |

坐标变换（`solver.cpp:164-178`）：

```text
xyz_G = R_cg · tvec + t_cg
xyz_W = R_gw · xyz_G                       // 世界系原点在云台中心，故无平移项
R_gw  = R_g2bᵀ · R(q) · R_g2b              // R_g2b = R_gimbal2imubody (solver.cpp:56)
ypr   = eulers(R, 2,1,0)  ⇔  R = Rz(yaw)·Ry(pitch)·Rx(roll)   // 已数值验证严格互逆
ypd   = ( atan2(y,x), atan2(z,√(x²+y²)), ‖xyz‖ )
```

单位：像素 px；长度 m；角度 rad（**下行帧里的 yaw/pitch/vel/acc 全部是 rad 制**，而 `io::GimbalState::yaw/pitch` 是**度**）。

时间戳：`camera.read(img, t)` 给出曝光时刻 `t`（含 `timestamp_offset_us` 修正）；云台姿态用 `gimbal.q(t)` 对 IMU 队列做 **slerp** 插值到同一时刻（`gimbal.cpp:98-112`），保证 `R_gw` 与图像同步。

---

## 1. 观测层：一帧到底拿到什么

检测器（`detection/ov_yolos/yolo11.cpp:236-261`）对每块装甲板输出 **4 个关键点**，并强制排序为

```text
points[0] = 左上   points[1] = 右上   points[2] = 右下   points[3] = 左下
```

由它构造 `Armor`（`armor.cpp:59-97`）：

```text
center = (p0+p1+p2+p3)/4
两条灯条：左灯条 = (p0, p3)（上端点、下端点），右灯条 = (p1, p2)
ratio  = max(上边长, 下边长) / max(左边长, 右边长)
rectangular_error = max(|∠(p3-p0) - roll - 90°|, |∠(p2-p1) - roll - 90°|)
```

这 4 个点此后有两条用途完全不同的路：

* **传统路（PnP）**：4 点 → solvePnP → 世界系位姿 → 4 维 ypd 观测；
* **UV 路（默认开启，`uv_observation: true`）**：4 点**不跑 PnP**，直接压成 8 维 UVL 观测（见 §3B）。

---

## 2. 四个像素点 → 三维位姿

### 2.1 物方 4 点

装甲板系下的 4 个角点（顺序与像素点一一对应，`solver.cpp:21-30`）：

```text
small: W = 0.135 m, big: W = 0.230 m, 灯条长 L = 0.056 m
P0 = (0, +W/2, +L/2)   P1 = (0, -W/2, +L/2)
P2 = (0, -W/2, -L/2)   P3 = (0, +W/2, -L/2)
```

即装甲板系的 x 轴是板面法线 —— 这正是"PnP 只能给出 yaw 的镜像二解"的根源。

### 2.2 solvePnP

```text
cv::solvePnP(object_points, points, K, dist, rvec, tvec, false, SOLVEPNP_IPPE)
```

IPPE 是平面 4 点解析解，无初值，给出两组解（选中的那组）+ 位姿：

```text
R_ac = Rodrigues(rvec)          // 装甲板→相机
R_aw = R_gw · R_cg · R_ac       // 装甲板→世界
ypr_in_world = eulers(R_aw, 2,1,0)
```

### 2.3 yaw 重投影搜索（消除镜像解）

PnP 的 yaw 在板面接近正对时不可靠，于是用"固定倾角 + 一维搜索"重定向（`solver.cpp:339-361`）：

```text
gimbal_yaw = eulers(R_gw,2,1,0)[0]
yaw0 = wrap(gimbal_yaw - 70°)                     // 搜索窗 ±70°（相机 FOV 级别）
for i = 0..139:                                   // 步长 1°
    yaw_i = wrap(yaw0 + i·1°)
    R_aw(yaw_i) = Rz(yaw_i)·Ry(pitch_armor)       // pitch_armor = +15°（前哨站 -15°），roll ≡ 0
    cost_i = Σ_{k=0..3} ‖ p_k - π_k(yaw_i) ‖      // π = projectPoints(R_cgᵀ R_gwᵀ R_aw, ...)
    best  = argmin cost_i
armor.yaw_raw = 原 PnP yaw;  armor.ypr_in_world[0] = best
```

> 注意：倾斜角只用固定值 ±15°，搜索目标是"四点重投影像素误差之和"；`SJTU_cost` 那条加权代价路径已被注释掉（`solver.cpp:403`）。

传统路的输出就是 `xyz_in_world`、`ypr_in_world`、`ypd_in_world` 三组量。

---

## 3. 观测进 EKF

### A. 传统 4 维观测（UV 关闭时）

```text
z = [ ypd_W[0], ypd_W[1], ypd_W[2], ypr_W[0] ]        // [方位, 俯仰, 距离, 板朝向]
R = diag(σ_az², σ_az², σ_d², σ_a²)                    // 配置：0.01 rad, 0.15 m, 0.10 rad
```

### B. UV 8 维观测（默认，每块板）

每条灯条压成 4 维 `(角度, 中心x, 中心y, 长度)`（`uv_model.cpp:238-264`）：

```text
dx = top.x - bottom.x, dy = top.y - bottom.y, len = √(dx²+dy²)
angle = atan2(dx, -dy)          // 竖直灯条 = 0，绕开 atan2 在 ±π 的回绕
c     = ((top+bottom)/2)        // px
len   = √(dx²+dy²)              // px，是差分量 ⇒ 对云台姿态共模误差免疫，是深度的主要信息来源
z_UVL = [左(angle,cx,cy,len), 右(angle,cx,cy,len)]     // 8 维
```

预测侧：把状态预测出的 4 个角点投影到像素，再压成同样的 UVL。角点世界坐标（`uv_model.cpp:105-115`）：

```text
θ_id   = wrap(x[6] + id·2π/N)                      // 该板的朝向
r_id   = (N==4 && id∈{1,3}) ? x[8]+x[9] : x[8]     // 4 板时 1/3 号板用带补偿的半径
p_id   = ( x[0] - r_id·cosθ_id ,  x[2] - r_id·sinθ_id ,  x[4] + Δh )
         Δh = (outpost) x[10]·mult(id) : (N==4 && id∈{1,3} ? x[10] : 0)
R_aw   = Rz(θ_id)·Ry(±15°)
corner_k = p_id + R_aw · local_k
P_cam  = R_cgᵀ (R_gwᵀ · corner_k - t_cg)
uv_k   = π(P_cam)   // 含 k1,k2,p1,p2,k3 畸变，要求 Z > 0.2 m
```

雅可比链（`uv_model.cpp:303-340`）：

```text
H = [ J_uvl(左) · [[J_corner0],[J_corner3]] ]      // 4×4 · 4×11 = 4×11
    [ J_uvl(右) · [[J_corner1],[J_corner2]] ]      // 合计 8×11
J_corner_k = J_proj(2×3) · R_world2camera(3×3) · J_world_k(3×11)
J_world_k  = ∂(p_id + R_aw·local_k)/∂x             // 中心、半径、高度、yaw 四项
```

噪声（`uv_model.cpp:349-382`）：

```text
σ_endpoint = √(σ_px² + (σ_len_ratio·len)²)        // σ_px = 1.0 px, σ_len_ratio = 0.02
σ_center   = √(σ_endpoint²/2 + σ_ind²)            // σ_ind 来自云台姿态误差的"独立"部分
σ_length   = σ_endpoint·√2                        // 差分量，不含共模平移
σ_angle    = √(σ_angle0² + 2σ_endpoint²/len²)     // σ_angle0 = 0.1 rad
R = diag(σ_angle², σ_center², σ_center², σ_length², ...)   // 8×8 两块
```

多板联合更新时把同帧 K 块板堆成 8K 维**一次**更新，并加 rank-3 姿态共模项（`attitude_common_ratio = 0` 时该项为 0）：

```text
z = [z_1; …; z_K],  H = [H_1; …; H_K],  R = blkdiag(R_k) + σ_com² · J_att J_attᵀ
```

关联用两级门限（`rv_from_fyt.cpp:272-323`）：可见性 `facing_cos ≥ 0.15` → 灯条中心像素距离 `min(d_left,d_right) ≤ max(60px, 3σ_px)` → 8 维马氏距离最小者；多板融合时第二块起还要过 `multi_armor_gate = 20`。

---

## 4. EKF 本体：11 维 RV-from-FYT

状态（`target.hpp` 注释 + `target.cpp:65-71`）：

```text
x = [ x, vx, y, vy, z, vz, yaw, vyaw, r, Δr, Δh ]ᵀ
     0   1   2   3   4   5    6     7    8   9  10
```

初始值由首帧 PnP 观测反推旋转中心：

```text
c_x = xyz_W[0] + r·cos(ypr_W[0])
c_y = xyz_W[1] + r·sin(ypr_W[0])
c_z = xyz_W[2]
x0 = [c_x, 0, c_y, 0, c_z, 0, ypr_W[0], 0, r, 0, Δh0]
P0 = diag(P0_dig)     // 默认 {1,64,1,64,1,64,0.4,100,1,1,1}
```

半径/板数按兵种（`tracker.cpp:606-631`）：默认 `r=0.2, N=4`；平衡/英雄大装甲 `r=0.2, N=2`；前哨站 `r=0.2765, N=3`；基地 `r=0.3205, N=3`。UV 模式下把半径先验收紧到 `uv_radius_prior_sigma²`。

**预测**（`rv_from_fyt.cpp:53-115`）：

```text
F = blkdiag( [[1,dt],[0,1]], [[1,dt],[0,1]], [[1,dt],[0,1]], [[1,dt],[0,1]], I₃ )
B = [dt²/2, dt, 0,0, 0,0, 0,0, 0,0, 0]ᵀ (对应 ax) 等三列
x⁻ = F x + B u ,  x⁻[6] = wrap(x⁻[6])
Q = blkdiag( Qb(σ_p), Qb(σ_p), Qb(σ_z), Qb(σ_yaw), 0₃ )
Qb(σ) = [[dt⁴/4·σ, dt³/2·σ], [dt³/2·σ, dt²·σ]]
普通目标 σ_p = 100, σ_yaw = 400；前哨站收敛后 σ_p = σ_yaw = 0.1（位置锁死），且 |x[7]| 钳到 2.51 rad/s
P⁻ = F P Fᵀ + Q
```

`u`（三轴加速度）平时为 0；开启中心加速度前馈时，`u_xy` 由最近 0.25 s 的 EKF 后验中心轨迹做**二次多项式最小二乘拟合**得到（`design = [1, Δt, Δt²/2]`，取二次项系数），再经 EMA(0.2)、|a| ≤ 6 m/s²、|jerk| ≤ 30 m/s³ 限幅（`center_acceleration_estimator.cpp:92-160`）。

**观测模型**（传统路，`rv_from_fyt.cpp:448-495`）：

```text
h(x,id) = [ ypd(p_id) , θ_id ]        // p_id、θ_id 同 §3B
H = H_ypda · H_xyza                   // 4×4 · 4×11
H_xyza 的非零列：
  ∂p/∂yaw = ( r·sinθ , -r·cosθ , 0 )
  ∂p/∂r   = ( -cosθ , -sinθ , 0 )     （1/3 号板同时进 ∂/∂Δr）
  ∂p/∂Δh  = mult(id) 或 1
  ∂θ/∂yaw = 1
H_ypd = xyz2ypd_jacobian(p_id)
```

**校正**（`extended_kalman_filter.cpp:71-129`）：

```text
r = wrap_angle(z - h(x⁻))             // 0、1、3 分量做 ±π 归一化
S = H P⁻ Hᵀ + R
K = P⁻ Hᵀ S⁻¹                         // 用 LDLT 解，非正定直接抛异常
x⁺ = x⁻ ⊕ (K r)                       // 角度分量加法后 wrap
P⁺ = (I-KH) P⁻ (I-KH)ᵀ + K R Kᵀ       // Joseph 形式，保正定
NIS = rᵀ S⁻¹ r，与 χ²_{0.95}(dim) 比较（4 维阈值 9.488）
```

关联（非 UV 路）同样用马氏距离 `rᵀ S⁻¹ r`，并带**迟滞**：若上一帧 id 的距离 < 9.488 且与最小距离相差 < 5.0，则保持上一帧 id（`rv_from_fyt.cpp:170-218`）。

**输出物**——目标模型给出的 4 块装甲板（`rv_from_fyt.cpp:411-421`）：

```text
for id = 0..N-1:
    xyza_list[id] = { p_id , θ_id }        // 每块板一个 (x, y, z, yaw)
```

这就是"目标的四个点"在滤波器里的形态：一个圆的刚体参数化（x, y, z, yaw, vyaw, r…），而不是四个独立点。滤波器发散判据 `r ∈ (0.05, 0.5)`；收敛判据 `update_count > 3`（前哨站 > 10）。

---

## 5. 从目标到瞄准角（Planner）

### 5.1 时间戳（最关键的一步）

`Planner::plan(optional<Target>, v, gimbal_yaw, rbSuppressiveFire)`（`planner.hpp:79-112`）：

```text
speed_delay = ( |x[7]| > 8 rad/s ) ? 0.030 s : 0.015 s      // 发弹→出膛
delay       = speed_delay + gimbal_delay (0.002)            // 云台提前量
future      = now + delay
target.predict(future)                                      // 纯预测，无观测更新
```

`Target::predict(dt)` 内部：`x ← F(dt)x + B(dt)·u`，前哨站把 `x[10]` 钉在 0.10 m。

### 5.2 选板

`get_recent_armor_xyzad()`（`target.cpp:402-452`）：

```text
主选：min over id of ‖(x_id, y_id)‖            // xy 平面最近
回锁：若 |vyaw| < 90°/s 且 wrap(θ_last - center_yaw) < 60°，改用 last_id 那块
输出：(x, y, z, yaw, d)      d = ‖(x,y)‖
```

### 5.3 弹道与飞行时间（含阻力）

```text
d_fix = d + target_dist_error (-0.23 m)
h_fix = z + target_h_error    (+0.28 m)
```

`tools::TrajectoryV2`（`trajectory.cpp:42-102`），阻力按**常值减速**近似（`v0 > 18` 判定为小弹丸）：

```text
k/m  = 6.7165e-5 / 0.0032   (小弹丸，v0 > 18 m/s)
     = 4.29838e-4 / 0.043   (大弹丸)
a    = (k/m)·v0²
disc = v0² - 2·a·d_fix
t_±  = (v0 ± √disc) / a          → 取较小的正根 = fly_time（平直弹道）
sinθ = (h_fix + ½·g·t²) / (v0·t) ,  g = 9.7833
pitch = asin(sinθ)               → 无解置 unsolvable（调用方抛异常，本帧不控制）
```

数值感（v0 = 22 m/s，小弹丸）：d = 3 m → t ≈ 0.141 s；d = 6 m → t ≈ 0.292 s；d = 10 m → t ≈ 0.516 s。

### 5.4 命中时刻

```text
target.predict(fly_time)          // 目标推进到 t* = now + delay + fly_time
yaw0   = wrap( atan2(y*, x*) + yaw_offset_ )              // 绝对方位角（弧度）
pitch* = asin(...) + pitch_offset_（远/远高各有一档补偿）
```

`t*` 就是"子弹打出去以后目标会在哪"的时刻——整条 MPC 轨迹以它为锚点。

---

## 6. 参考轨迹：100 拍（1 s），第 50 拍 = 命中时刻

`rbget_trajectory`（`planner_trajectory.cpp:31-53`），DT = 0.01 s，HORIZON = 100：

```text
target.predict(-DT·(HALF_HORIZON+1)) = t* - 0.51         // 回退
θ(-0.51), θ(-0.50) 由 rbaim 给出

for i = 0..99:                                            // col(i) 对应 t* - 0.50 + i·DT
    target.predict(DT)
    θ_next = rbaim(target)                                // 该时刻的瞄准 yaw/pitch
    ω(i) = wrap(θ_next - θ_last) / (2·DT)                 // 中心差分（20 ms 窗）→ 抗板间跳变噪声
    traj.col(i) = [ wrap(θ_now - yaw0), ω(i), pitch_now, ωp(i) ]
    θ_last, θ_now = θ_now, θ_next
```

时间轴对齐关系（这是"第 50 拍 = 开火时刻"的由来）：

```text
col 0  ← t* - 0.50 s        col 50 ← t*        col 99 ← t* + 0.49 s
```

参考轨迹已经把弹道俯仰、装甲板相位、目标自转全部含进去了，因此它是一条**角度随时间变化的目标曲线**，不是一条直线。

---

## 7. MPC：把参考轨迹变成 (θ, ω, α)

每个轴（yaw / pitch 各一个 TinyMPC 求解器，`planner_mpc.cpp`）：

```text
模型： x_{k+1} = A x_k + B u_k ,   x = [θ, ω]ᵀ
       A = [[1, DT],[0, 1]],  B = [[0],[DT]] ,  f = 0    // 离散双积分器
目标： min Σ_{k=0}^{N-1} (x_k - x_ref,k)ᵀ Q (x_k - x_ref,k) + Σ_{k=0}^{N-2} u_kᵀ R u_k
约束： |u_k| ≤ u_max
参数： N = 100, ρ = 1.0, ADMM max_iter = 10
      yaw  : Q = diag(9e6, 0), R = 1, u_max = 50  rad/s²
      pitch: Q = diag(9e6, 0), R = 1, u_max = 100 rad/s²
初值： x0 = traj.col(0)   ← 取参考轨迹起点，**刻意不使用云台反馈**
```

`Q` 的第二项为 0 意味着：不惩罚速度偏差，只惩罚角度偏差；ω 由"最小加速度代价把 θ 追上参考"自然决定。Q/R = 9e6 使角度跟踪极硬，实际解几乎是"用最省加速度的方式贴合参考角曲线"。

求解得到：

```text
x = 2×100（θ_k, ω_k）      u = 1×99（α_k）
x_{k+1} 的展开 ⇒  α_k = (θ_{k+2} - 2θ_{k+1} + θ_k)/DT²   （角度序列的离散二阶差分）
```

---

## 8. 下行帧：最终发给下位机的量

`write_mpc_commands`（`planner.cpp:884-904`）取**时域中心拍** idx = HALF_HORIZON = 50（= 命中时刻 t*）：

```text
plan.yaw       = wrap( x_yaw(0,50)   + yaw0 )     // rad，世界系绝对偏航角
plan.yaw_vel   =       x_yaw(1,50)                // rad/s，MPC 速度状态
plan.yaw_acc   =       u_yaw(0,50)                // rad/s²，MPC 控制量（≤ 50）
plan.pitch     =       x_pitch(0,50)              // rad，绝对俯仰角（参考里已含弹道角+补偿）
plan.pitch_vel =       x_pitch(1,50)              // rad/s
plan.pitch_acc =       u_pitch(0,50)              // rad/s²（≤ 100）
plan.control   = true
plan.fire      = 见 §9
```

开火判据与它同拍（这也是 `shoot_offset` 存在的意义）：

```text
Dynamics 策略： plan.fire = hypot( traj(0,50+off) - x_yaw(0,50+off),
                                   traj(2,50+off) - x_pitch(0,50+off) ) < fire_thresh (0.003 rad)
               off = shoot_offset = 2  →  在 t* + 20 ms 处检查"参考 vs MPC"的跟踪残差
```

打包发送（`gimbal.cpp:155-169`，`gimbal.hpp:43-55`）：

```text
mode = control ? (fire ? 2 : 1) : 0
frame = [head=0x66][mode][yaw][yaw_vel][yaw_acc][pitch][pitch_vel][pitch_acc][crc16 LE][end=0x11]
        = 1 + 1 + 4×6 + 2 + 1 = 29 字节，CRC16 只覆盖前 26 字节，460800 bps
```

发送节奏：`plan_thread` 每 10 ms 一次（100 Hz），相机约 90 fps；两次相机帧之间目标只做**预测**不做校正，因而下行的 (θ,ω,α) 主要随 `now` 前移和重规划而变化。

---

## 9. 步兵路（rbSuppressiveFire）的开火判据

`is_fire`（`planner.cpp:510-568`）：以"当前云台指向"是否落在装甲板张角窗内为准。

```text
参考目标 = target.predict(-gimbal_delay)      // 回退到真正的命中时刻 now+speed_delay+fly
选择的板：(x, y, z, yaw) ← get_recent_armor_xyzad()
半宽 S = 0.12 m（小装甲）/ 0.22 m（大装甲）/ 0.10 m（前哨站、基地）

左边缘 L = (x - 0.5·S·sinψ,  y + 0.5·S·cosψ)
右边缘 R = (x + 0.5·S·sinψ,  y - 0.5·S·cosψ)      // 沿板面切向偏移半个板宽
center_angle = atan2(y, x)
d_L = wrap(atan2(Ly,Lx) - center_angle) = -0.5S/d 量级
d_R = wrap(atan2(Ry,Rx) - center_angle)
delta = wrap( (gimbal_yaw/57.3 - yaw_offset_) - center_angle )

fire = ( d_min ≤ delta ≤ d_max )  &&  pitch 条件  &&  前哨站稳定条件
```

前哨站额外约束（`planner.cpp:472-506`）：`vz > 0.09` 或 `update_count < 500` 时禁火，并改用旋转中心前向点 `(c_x - r·cos(dir), c_y - r·sin(dir), z)` + 0 速度 0 加速度的"接管角"。

---

## 10. 端到端数值例子（步兵，小弹丸）

设：目标 6 m、`vyaw = 4 rad/s`、`r = 0.2 m`、与云台等高、`bullet_speed = 22 m/s`。

| 步骤 | 结果 |
| --- | --- |
| 弹道 | `a = 10.159 1/s`，`disc = 484 - 121.9 = 362.1`，`t = (22-19.03)/10.159 = 0.2925 s` |
| 命中时刻 | `t* = now + (0.015 + 0.002) + 0.2925 = now + 0.3095 s` |
| 参考轨迹 | col 0 = now − 0.20 s，col 50 = t*，col 99 = t* + 0.49 s |
| 装甲板切向速度 | `v_t = r·ω = 0.8 m/s` |
| 云台角速度量级 | `v_t/d ≈ 0.8/6 = 0.133 rad/s`（7.6°/s，远小于 50 的上限） |
| 云台角加速度量级 | `ω²r/d = 16×0.2/6 = 0.53 rad/s²`（远小于 50） |
| 下行帧 | `yaw ≈ 目标相位对应绝对角`，`yaw_vel ≈ 0.13`，`yaw_acc ≈ 0.5`，`pitch ≈ 3.7°→0.065 rad` |

即：下位机拿到的是"0.31 s 之后应当指向的绝对角 + 该时刻的角速度 + 该时刻的角加速度"，由它自己的电流环/速度环去跟踪。

---

## 11. 七个容易踩的点（供 review 用）

1. **α 不是对 θ 做数值微分**：它是 MPC 的控制量 `u`，与 `(θ_{k+2}-2θ_{k+1}+θ_k)/DT²` 恒等，天生满足离散双积分一致性，不含差分噪声。
2. **MPC 不使用云台反馈**（`planner.cpp:110-112` 的注释是刻意设计）：初值取参考轨迹起点，命令完全由目标决定。云台反馈 `gimbal_yaw` 只喂给开火判据。好处是命令不随反馈抖动；代价是"绝对角精度"依赖下位机自己的闭环与标定。
3. **只有 θ 是绝对量，ω/α 是 MPC 内部量**；且三者都取自同一拍（idx = 50），保证"发的"和"判的"是同一时刻。
4. **单位陷阱**：`io::GimbalState.yaw/pitch` 是**度**；下行帧是**弧度**；`rbplan` 里的 `plan.target_yaw` 是**度**（仅 debug 用），而 `sbplan` 里是**弧度**。
5. **符号陷阱**：`Aimer::aim`（旧路径）返回 `pitch = -(traj.pitch + offset)`（"向上为负"），而 `Planner::aim/rbaim` 返回 `+traj.pitch`（向上为正）。步兵/英雄主程序走 Planner，哨兵全向路径仍可能走 Aimer，两者俯仰符号相反。
6. **参考轨迹只用 1 块板**：`rbaim` 只取最近（或锁定）的那块板；另外 3 块板用于选板、开火窗、前哨站高度阶梯与 pitch 判别、以及 `heroaim` 的旋转中心前向点。
7. **参数改动会影响 MPC 语义**：`Q = diag(9e6, 0)` 中第二项为 0 表示"速度不加权"；若把它改成非 0，MPC 就会开始惩罚速度偏差，`yaw_vel` 的数值含义随之改变（不再是"最省加速度解出的速度"）。

---

## 12. UVL 四个分量的几何来源，以及"EKF 到底假设了什么"

### 12.1 每个分量来自灯条的哪个位置

UVL 只用了装甲板 4 个角点里的**每条灯条的 2 个端点**（`uv_model.cpp:238-264`）：

| 装甲板关键点 | 归属 | UVL 分量 | 公式 | 物理含义 |
| --- | --- | --- | --- | --- |
| `points[0]` 左上 → top | 左灯条 | `z0` 角度 | `atan2(dx, -dy)` | 灯条**轴向相对图像竖直方向**的倾角（竖直灯条 = 0；上端偏右为正） |
| `points[3]` 左下 → bottom | 左灯条 | `z1`,`z2` 中心 | `((top.x+bottom.x)/2, (top.y+bottom.y)/2)` | **灯条线段的像素中点**（= 装甲板左边缘中点），不是 `Armor::center`（4 点均值），也不是旋转矩形的 `Lightbar::center` |
| | | `z3` 长度 | `√(dx²+dy²)`，`dx=top.x-bottom.x, dy=top.y-bottom.y` | 灯条在图像上的**视长度**（px） |
| `points[1]` 右上 → top | 右灯条 | `z4..z7` | 同上 | 装甲板右边缘中点 / 长度 / 倾角 |

模型侧的对应物（`uv_model.cpp:61-68`）：`armor_corner_local = (0, ±W/2, ±L/2)`，即"灯条"被建模成**距装甲板中心 ±W/2、长 L = 0.056 m 的板边缘线段**；左灯条 = 局部点 0/3，右灯条 = 1/2。所以预测的中心是"板边缘中点的投影"，长度是"0.056 m 线段在该姿态下的投影长度"。

两个由构造决定的性质：

* **差分量免疫共模**：`长度`是两端点之差，云台姿态误差造成的整帧平移不进入该分量 ⇒ 它承担了几乎全部深度信息。
* **镜像歧义**：无倾角时，法线翻转 180° 的 4 个投影点集合完全相同（矩形对称），UVL 完全不变；正是因为模型里带了 **±15° 倾角先验**，这个对称性被弱破坏，yaw 才可观测（`tests/uv_jacobian_test.cpp:230` 断言"tilt prior breaks the 180deg symmetry"，`:217/:221` 断言角度与长度都携带 yaw 信息）。

### 12.2 EKF 的三条假设是分开的

```text
模型：x_{k+1} = f(x_k) + w_k ,  w ~ N(0,Q)
      z_k     = h(x_k) + v_k ,  v ~ N(0,R)
```

| 假设 | 作用是 | 不满足会怎样 |
| --- | --- | --- |
| 噪声**零均值** + 二阶矩（协方差）已知 | 决定 `K = P⁻HᵀS⁻¹`、`P⁺ = (I-KH)P⁻(I-KH)ᵀ+KRKᵀ`。此时 KF 是**最佳线性无偏估计**（LMMSE），**不需要高斯** | 有偏噪声（标定误差、模型误差）会被当成状态误差吸收 ⇒ 估计一致但有偏 |
| 噪声**高斯** | 让 KF 成为贝叶斯 MMSE 最优、让 `P` 等于真实后验协方差、让 `NIS` 严格服从 χ² | 仅"最优性/门限的统计意义"退化；门限变成经验值 |
| `f`、`h` **线性** | 让后验保持高斯、KF 递归精确 | EKF 的做法：一阶泰勒展开，**噪声假设完全不动** |

**关键结论：线性化作用在函数 `f`/`h` 上，不作用在噪声分布上。** 它不会把非高斯噪声变成高斯，也不会把有偏误差变成零均值——那两件事只能靠改噪声模型（`Q`/`R`、增广状态、鲁棒代价）解决。

代码里已经做对的两件事（很多实现反而做错）：

* **均值走精确非线性**：`predict(F,Q,f)` 用 `x = f(x)` 而不是 `F·x`（`extended_kalman_filter.cpp:59-61`）；`update` 的残差用真实 `h(x_prior)` 而不是 `H·x_prior`（`:79`）。线性化只用于协方差与增益。
* **角度在切空间处理**：`predict` 里 `x[6]=wrap`、`state_add` 里 wrap、`observation_subtract` 里对 0/1/3 分量 wrap（`rv_from_fyt.cpp:509-528`）。这是误差状态/流形 EKF 的做法，避免了 yaw 在 ±π 附近"线性化到错误的一侧"。

### 12.3 一阶线性化丢掉了多少（可量化）

```text
均值：  E[h(x)] ≈ h(x̂) + ½·tr(∇²h · P)      ← EKF 丢掉的第二项
协方差：P⁺ 是"线性化系统"的精确后验协方差；对真实非线性系统的误差同为二阶量
判据：  二阶项 / 一阶项 ≈ 状态不确定度 / 特征尺度
```

对 UVL 的两个通道分别算：

```text
长度通道： len ≈ f·L/d ⇒ len''/len' = 2/d，二阶项/一阶项 ≈ σ_d / d
           σ_d/d = 10%（6 m 处 σ_d = 0.6 m）→ 二阶项占 10%，可忽略
           σ_d/d = 30%                        → 进入边缘区
中心通道： 像素坐标 ∝ 1/d 且含 r·cosθ 乘积项 ⇒ 二阶项相对量 ~ σ_yaw (rad)
           收敛后 σ_yaw ~ 几度 → 没问题；初始 P0[6] = 0.4 (σ_yaw = 36°) → 一阶展开不成立
```

代码对此的三处"兜底"：

* `uv_radius_prior_sigma = 0.02` 把半径先验从默认 `P0[8] = 1`（σ_r = 1 m）收紧到 σ_r = 0.02 m —— 半径/深度方向是弱可观测方向，先验不放小，线性化误差和深度漂移都会失控；
* `uv_process_noise_z_scale` 给深度单独缩放过程噪声；
* 像素关联门限取 `max(60 px, 3σ_px)`，即状态越不确定门限越宽，避免"线性化不准 ⇒ 残差大 ⇒ 永远关联不上"。

已有的测试只证明了**切平面算得对**（解析雅可比 vs 中心差分 ≤ 1e-4，`uv_jacobian_test.cpp:167`；前向模型 vs `cv::projectPoints` ≤ 1e-3 px，`:137`），**没有**证明切平面是好的近似——后者取决于 `P` 的大小，只能靠 NIS 在线判断。

### 12.4 本工程里真实噪声偏离高斯的地方

| 环节 | 真实分布 | 代码的应对（都只是"二阶矩"层面的近似） |
| --- | --- | --- |
| 关键点回归误差（~1 px） | 近似高斯、轻微重尾 | `σ_px = 1.0`，另加与像素长度成比例的 `σ_len_ratio·len`（异方差） |
| 云台/外参姿态误差（0.5°） | **系统性、整帧共模** | 折算成像素：`σ_att_px = ½(fx+fy)·σ_att = 15.8 px`，加到中心通道；`attitude_common_ratio=0` 时按**独立对角**处理，`=1` 时用 rank-3 的 `J_attJ_attᵀ` 共模项。两种都只是二阶矩模型 |
| 灯条角度 `atan2(Δx,−Δy)` | `len` 小时**重尾**（两个小量之比） | `σ_angle² = σ_angle0² + 2σ_ep²/len²`（一阶传播）⇒ len 越小越不信任 |
| 灯条长度（欧氏范数） | Rician，向上偏 ~σ²/len | 未显式建模；`len ≫ σ` 时可忽略 |
| PnP 深度（传统路） | 重尾/偏斜，随距离增长 | 启发式 `σ_d² = log(|Δθ|+1)+1`；配置直接改成常数 0.15 m |
| 误检、串车、遮挡 | 重尾离群 | 马氏门限 9.488、多板门限 20、`exclude_ids`、迟滞 5.0、NIS 失败率 > 40% ⇒ 丢目标 |
| 目标机动（加速度非白） | 有色、非高斯 | 过程噪声 `σ_p=100, σ_yaw=400`（很松）+ 中心加速度前馈（0.25 s 二次拟合 + EMA + jerk 限幅） |
| 手调补偿 `−0.23 m / +0.28 m / pitch_offset` | 纯偏置 | `R` 表达不了，直接变成 EKF 状态偏置 |

数值对比很说明问题（`len = 20 px`，`fx≈fy≈1813`，`σ_att = 0.5°`）：

```text
σ_endpoint = √(1² + (0.02·20)²) ≈ 1.08 px
σ_length   = σ_endpoint·√2      ≈ 1.52 px      ← 被信任
σ_center   = √(σ_ep²/2 + 15.8²) ≈ 15.85 px     ← 几乎不被信任
σ_angle    = √(0.1² + 2·1.08²/20²) ≈ 0.126 rad (7.2°)
```

即：**每帧真正被信任的只有"长度"（深度）和很弱的"角度"**，方位主要靠时间连续性与运动模型累积 —— 这正是 UV 路能用但单帧方位精度不高的数学原因。

### 12.5 想更进一步的选择

| 方案 | 解决什么 |
| --- | --- |
| UKF / sigma 点 | 二阶项自动进入均值与协方差，无需解析雅可比（UV 的解析雅可比就白写了） |
| IEKF（迭代 EKF） | 等价于对 MAP 做 Gauss-Newton，显著减小强非线性下的偏差（近距/初始几帧） |
| 误差状态 EKF | 角度已在用；把半径/深度也放进误差状态可进一步解耦 |
| 状态增广相关噪声 | 把云台姿态误差（整帧共模）建成状态而不是噪声 —— 对应 `attitude_common_ratio = 1` 那条路，配置目前是 0 |
| 鲁棒滤波（Huber / Student-t） | 用连续降权替代"硬门限 + 直接丢弃"，对重尾离群更稳 |
| 在线一致性监控 | 已有的 NIS/NEES 就是；若 NIS 长期系统性超标，说明是 `R` 太小或模型有偏，而不是"噪声不高斯" |

---

## 附：关键常数速查

| 常数 | 值 | 位置 |
| --- | --- | --- |
| DT / HALF_HORIZON / HORIZON | 0.01 s / 50 / 100 | `planner.hpp:17-19` |
| Q_yaw, Q_pitch | diag(9e6, 0) | `configs/rb_auto_aim.yaml:170-174` |
| R_yaw, R_pitch | 1 | 同上 |
| max_yaw_acc / max_pitch_acc | 50 / 100 rad/s² | 同上 |
| fire_thresh / shoot_offset | 0.003 rad / 2 | 同上 |
| high/low_speed_delay_time | 0.030 / 0.015 s | 同上 |
| decision_speed | 8 rad/s | 同上 |
| gimbal_delay | 0.002 s | 同上 |
| target_dist_error / target_h_error | −0.23 m / +0.28 m | 同上 |
| 装甲板容差 | 0.12 / 0.22 / 0.10 m | 同上 |
| 灯条长 / 板宽 | 0.056 m / 0.135 m / 0.230 m | `solver.cpp:17-19` |
| 弹道 g / k / m | 9.7833 / 6.7165e-5,0.0032 / 4.29838e-4,0.043 | `trajectory.cpp:8,37-40` |
| EKF 观测噪声 | 0.01 rad, 0.15 m, 0.10 rad | `configs/rb_auto_aim.yaml:58-60` |
| UV 噪声 | 1.0 px, 0.02, 0.5°, 0.1 rad | `configs/rb_auto_aim.yaml:71-79` |
