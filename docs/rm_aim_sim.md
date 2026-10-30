# 接入 rm_aim_sim（RoboMaster Aim Lab）仿真

本文说明如何把本仓库的自瞄接进 `~/Desktop/rm_aim_sim` 的共享内存仿真，并给出已经跑通的构建、启动、验收和调参步骤。

## 1. 和现有 `rb_auto_aim_simulator.cpp` 的区别

| | `rb_auto_aim_simulator.cpp` | 本文的 `rm_aim_sim_client.cpp` |
|---|---|---|
| 输入 | ROS 2 话题 `/image_raw`、`/camera_info`、`/tf` | `/dev/shm/rm_aim_sim_<channel>/` 文件邮箱 |
| 依赖 | rclcpp、sensor_msgs、tf2 | 无 ROS，只依赖 OpenCV/Eigen/fmt/yaml-cpp/nlohmann-json |
| 输出 | 只显示/画图，不直接回控 | 写 `command.json`，直接控制仿真云台和开火 |
| 适用 | 另一个 ROS 2 仿真 | 本机 rm_aim_sim |

两者共用同一套 `tasks/auto_aim` 算法（Solver / Tracker / Planner），区别只在“相机”和“云台通信”两处适配层。原有 ROS 2 入口保持不变。

新增文件：

```text
src/rm_aim_sim_client.cpp   共享内存客户端 + 检测/跟踪/规划/控制线程
configs/rm_aim_sim.yaml     仿真专用配置（相机内参、外参、检测器、MPC）
CMakeLists.txt              rm_aim_sim_client 目标
docs/rm_aim_sim.md          本文
```

## 2. 构建

```bash
cd ~/Desktop/sp_vision_25_rbclone
cmake -S . -B build
cmake --build build --target rm_aim_sim_client -j8
```

生成 `build/rm_aim_sim_client`。配置里的模型路径是相对路径（`../assets/...`），所以**运行目录必须是 `build/`**，和其他自瞄程序一致。

## 3. 启动

终端一，启动仿真并打开本机控制：

```bash
cd ~/Desktop/rm_aim_sim
./run.sh -- --remote
```

第一次运行前，如果还没导入资源，先按仿真仓库 README 执行一次：

```bash
./run.sh --headless --editor --import --quit
```

终端二，先只瞄准、不开火：

```bash
cd ~/Desktop/sp_vision_25_rbclone/build
./rm_aim_sim_client ../configs/rm_aim_sim.yaml
```

确认画面里出现检测框、底部状态显示 `target YES / tracker tracking`，且实际 yaw/pitch 跟规划值接近后，再加 `--fire`：

```bash
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --fire --duration=20
```

按 `q` / `Esc` 或 `Ctrl+C` 退出。退出时会尽力发一包 `control=false, fire=false`。

> 仿真必须处于**图形模式**：`--headless` 只跑物理和状态，没有像素输出，本程序会一直报“等待图像帧超时”。
> 不要同时运行 `tools/camera_demo.py`、`tools/control_demo.py` 或另一个自瞄：同一通道的 `client.lock` 是独占的。

## 4. 它怎么工作

### 4.1 数据流

```text
request.json ──> 仿真渲染一帧
                    │
frame.json + frame-*.raw（RGB/RGBA，1440x1080）
                    │
              mmap/读取 + RGB→BGR
                    │
        3x3 高斯模糊（仅检测器输入）
                    │
      traditional Detector / YOLO ──> Armor
                    │
       Solver：PnP + R_gimbal2world + 相机内外参
                    │
       Tracker：EKF（世界系目标状态）
                    │
       Planner：MPC + 弹道 + 开火判据
                    │
command.json（100 Hz，yaw/pitch/vel/acc/fire）
                    │
              仿真二阶云台 → 出膛 → 命中/扣血
```

线程划分：

- **采图线程**：发请求 → 收图 → 解码 → 入队。上一帧复制完才发下一次请求，不会在仿真侧积压图像。
- **感知线程**：检测 + PnP/跟踪，只更新目标快照；算法对象在本线程内构造和使用。
- **控制线程**：固定 100 Hz 读 `state.json`、调用 Planner、写 `command.json`；不等待图像或显示。
- **主线程**：显示、计时、退出。

### 4.2 坐标与符号

- 仿真世界系：x 前、y 左、z 上；yaw 左正，**pitch 向下为正**。
- `frame.json` 的 `gimbal.quaternion_wxyz` 直接作为 `R_gimbal2world` 喂给 `Solver::set_R_gimbal2world_from_tf()`。
- 每帧的 `camera.K`、`camera.distortion`、`camera.R_camera2gimbal`、`camera.t_camera2gimbal` 覆盖配置，分辨率/视场角在仿真界面改动后无需改 YAML。
- 本仓库 `Solver` 的世界原点是**云台旋转中心**；仿真世界原点是地面。两者 z 差 0.65 m，但 `Target`/Planner 都在云台系里解算，**不要**手动减 `gimbal.origin`。
- `Planner::plan.pitch` 来自 `tools::Trajectory`，**抬头为正**；下发仿真时取反：`pitch_sim = -plan.pitch`。yaw 同号。
- 仿真开火延迟、弹速、重力、射频都从 `state.weapon` 取；弹速默认 23 m/s。

### 4.3 2 px 灯条的来源：分辨率 vs 模糊

先说结论：**2 px 主要是角分辨率/投影尺度造成的**，不是模型或贴图问题。

默认仿真相机是 1440×1080、垂直 FOV 45°：

```text
fx = fy = 1080 / (2*tan(45/2)) = 1303.7 px
56 mm 长的灯条在 4 m 处约 18 px 长
灯条宽度约 5–6 mm，投影后只有 1.6–2 px
```

而且仿真默认的 `fx≈1304` 还低于本仓库实车配置的 `fx≈1813`（等效垂直 FOV≈33°）。也就是说，默认仿真画面比实车相机更“广角、低角分辨率”，灯条自然更细。

干净合成渲染又缺少真实相机的光学低通、JPEG/传感器噪声，1–2 px 宽的灯条边缘会严重走样。两条路都能解决：

1. **提高仿真相机的角分辨率（更根本）**：例如 2560×1920、FOV 45° → `fx≈2318`，灯条宽约 3 px；
2. **对检测输入做低通**：1440×1080 + 3×3 高斯模糊，把 2 px 灯条摊成更平滑的亮带。

实测（同一静止靶、曝光 0.4、`--fire-tolerance=1.0`）：

| 相机配置 | fx | traditional，blur=0 | yolo，blur=0 |
|---|---:|---|---|
| 1440×1080 / 45° | 1303.7 | 0 检测 | 0 检测 |
| 1920×1440 / 45° | 1738.2 | 0 检测 | 1 块，可跟踪 |
| 2560×1920 / 45° | 2317.6 | 1 块，稳定跟踪 | 1 块，稳定跟踪 |
| 1440×1080 / 45° + blur3 | 1303.7 | 1 块，稳定跟踪 | 1 块（关键点仍抖） |

`2560×1920 + traditional + blur=0` 开火实测 **23/23 = 100%**，约 1 个弹匣击毁。

几点解释：

- YOLO 的输入固定 416/640，整图缩放后目标的**相对**大小不会因为分辨率提高而变大；它受益的是“原始渲染更干净、降采样更平滑”，所以 1920×1440 就够它检出。
- `traditional` 直接处理原图像素，需要灯条宽度约 3 px 才稳定，所以要么 **2560×1920 + blur0**，要么 **1440×1080 + blur3**；1920×1440 也能检出但会间歇丢帧，不推荐给 traditional。
- 提高分辨率的代价是每帧字节和渲染/读回/拷贝开销：1440×1080 RGBA 约 6.2 MB、40–80 fps；2560×1920 RGBA 约 19.7 MB、约 30 fps。

所以默认仍是 `--blur=3`，目的是让仿真仓库**原样的 1440×1080 配置开箱可用**；如果你愿意改仿真相机，优先 2560×1920 + `--blur=0`，PnP 的角点精度也更好。

至于默认检测器：`traditional` 用灯条轮廓/PnP，静止和运动目标都更稳、约 1–2 ms；YOLO 在 1440×1080 + blur3 下虽然能检出，但 4 个关键点有 1–3 px 抖动，单装甲下 EKF 的 `vyaw` 会明显跳，跟踪不稳。更高的仿真分辨率会缓解这一点，但本次闭环默认仍用 `traditional`。

```bash
--detector=traditional   # 推荐
--blur=3                 # 1440×1080 默认；2560×1920 建议 0
--color-filter=false     # 颜色不可靠时仍能跟踪；单靶场景不需要颜色过滤
```

`--detector=yolo` 仍然可用，适合你已经有**在仿真外观上训练/微调过**的模型；把 `configs/rm_aim_sim.yaml` 里的 `yolov8_model_path` 指向该模型即可。

`--blur` 不是“作弊”读真值：它只是对输入像素做对称低通，不改变目标位置，也不使用 `camera.armors` 理想角点。`--show-ideal` 只用于把理想角点画出来做 PnP 对比，不参与控制。

### 4.4 默认内参、实车内参与分辨率

仿真相机是零畸变针孔模型，`scripts/simulation.gd` 里直接算：

```text
fy = 0.5 * height / tan(vertical_fov / 2)
fx = fy
cx = width / 2
cy = height / 2
```

默认 `[camera] width=1440, height=1080, vertical_fov=45°`，所以：

```text
fx = fy = 1303.6753236814714 px
cx, cy = 720, 540
hfov = 57.82°, vfov = 45°, dfov = 69.24°
distortion = [0, 0, 0, 0, 0]
```

**这是像素焦距**。仿真不定义传感器尺寸，所以没有唯一的物理 mm 焦距；只有在你假定像元尺寸后才有意义。例如按常见的 3.45 µm 像元（1/2.9" 1440×1080 全局快门）估算：`f ≈ 1303.675 × 3.45 µm ≈ 4.50 mm`，35 mm 等效（按对角线）约 **31 mm**。

本仓库实车配置 `configs/rb_auto_aim.yaml`：

```text
fx, fy = 1813.554, 1813.369
cx, cy = 712.275, 526.918
distortion = 5 项非零
```

YAML 本身不写分辨率，分辨率由相机原生输出决定。按主点接近 `720/540` 推断，标定图是 **1440×1080**（cx/cy 分别偏中心 −7.7/−13.1 px，属于正常标定偏差）。在 1440×1080 下：

```text
hfov = 43.31°, vfov = 33.17°, dfov = 52.79°
35 mm 等效（对角线）≈ 43.6 mm
```

所以仿真默认的 `fx≈1304` 比实车 `fx≈1813` 广 **1.39×**；同一个 4 m 目标在仿真里确实更小。要让仿真匹配实车角分辨率，二选一：

| 目标 | 仿真配置 | fx=fy | 备注 |
|---|---|---:|---|
| 保持 1440×1080 | `vertical_fov=33.17°` | 1813.37 | 分辨率不变，视场变窄 |
| 保持 45° | 约 `2002×1502`（用 2000×1500 即可） | 1810.66 | 像素更多，画面比例不变 |
| YOLO 可用 | 1920×1440 / 45° | 1738.23 | 实测 YOLO 不 blur 可跟踪 |
| traditional 稳定 | 2560×1920 / 45° | 2317.65 | 实测 traditional 不 blur 稳定，23/23 |

注意：共享内存里的 `frame-*.raw` 和右上角相机预览才是自瞄输入，始终是完整相机分辨率；**第一人称主画面是窗口缩放/裁边后的显示**，不要拿它测灯条像素宽度。

### 4.5 用一组实车内参反推仿真相机（示例）

例如这组实车内参：

```text
fx, fy = 2370.249, 2373.391
cx, cy = 683.697, 529.924
distortion = [-0.04247, 0.31838, -0.00188, -0.00387, -0.74188]
```

内参本身不含分辨率，先确认标定图尺寸（看标定程序里的 `frame.cols/rows`）。按主点推断：

| 标定图 | 主点偏移（相对中心） | 要设的 `vertical_fov` | 仿真 fx=fy | hfov | 35mm 等效 |
|---|---:|---:|---:|---:|---:|
| 1440×1080 | (−36.3, −10.1) px | **25.6522°** | 2371.82 | 33.77° | ≈57 mm |
| 1360×1024 | (+3.7, +17.9) px | 24.3628° | 2371.82 | 32.00° | ≈60 mm |
| 1280×1024 | (+43.7, +17.9) px | 24.3628° | 2371.82 | 30.20° | ≈63 mm |

以 1440×1080 为例，仿真配置：

```ini
[camera]
width=1440
height=1080
vertical_fov=25.6522
brightness_gain=0.4
# 如果有实车手眼平移，把默认 (0.08, 0, 0.04) 改成实车 t_camera2gimbal
# offset=Vector3(tx, ty, tz)
```

实测：该配置下 `frame.json` 的 `K=[2371.817,0,720, 0,2371.817,540, 0,0,1]`，与实车 fx/fy 差约 0.13%；4 m 静止靶用 `--detector=traditional --blur=0` 稳定跟踪，开火 **24/24 = 100%**，350 伤害击毁。

不能只靠仿真配置精确复现的部分：

- **主点 cx/cy**：仿真的主点固定为 `(width/2, height/2)`，无法单独设置。实车 1440×1080 下 cx/cy 偏中心 (−36.3, −10.1) px，对应约 yaw 0.87°、pitch 0.24° 的画面平移。中心目标影响不大；但**不要拿实车 K 直接套仿真图**，适配器用 `frame.json` 里的仿真 K 才是自洽的。
- **畸变**：仿真永远输出零畸变针孔图。按这组系数，目标在中心附近（归一化半径 0.1）偏差 <0.5 px，0.3 半径约 3.7 px，角点可达 20 px 以上。中心靶测试可忽略；要做边缘靶/全图识别对比，需要给仿真图像加畸变后处理，并让适配器改用实车 K，这不是改一个配置能完成的。
- **视场变窄后的可用范围**：4 m 处水平半宽只有 `4*tan(16.886°)≈1.21 m`。仿真自带的平移场景是 ±1.5 m，会跑出画面；做平移测试时把 `travel` 降到 1.0 左右，或把靶车放到 5–6 m。
- **相机安装旋转**：仿真只支持 `[camera] offset` 平移，`R_camera2gimbal` 固定为理想光学轴；实车手眼标定里的安装角不能通过 YAML 设置。
- **曝光/增益/gamma**：仿真的 `brightness_gain` 只是简化曝光，不等于实车的曝光时间/gain/gamma；图像亮度要单独调，和 K 无关。

## 5. 命令行选项

角度参数除特别说明外，命令行里都是**度**，内部转弧度。带值的选项建议写成 `--key=value`，布尔开关直接写 `--fire`、`--no-display`。

| 选项 | 默认 | 说明 |
|---|---|---|
| `config-path`（位置参数） | `../configs/rm_aim_sim.yaml` | YAML 配置 |
| `--channel` | `default` | 必须与仿真 `[ipc].channel` 一致 |
| `--fire` | false | 允许开火；默认只瞄准 |
| `--no-display` | false | 不开调试窗口；仿真仍须图形模式 |
| `--duration` | 0 | 运行秒数，0 表示一直运行 |
| `--detector` | `traditional` | `traditional` / `yolo` |
| `--blur` | 3 | 检测前高斯核，奇数；0 关闭 |
| `--color-filter` | false | 是否按 `enemy_color` 过滤 |
| `--fire-tolerance` | 1.0 | 实际云台与规划角的开火对齐容差（度） |
| `--hold-ms` | 80 | 丢目标后保持最后瞄准角的时间 |
| `--target-timeout-ms` | 150 | 目标过期时间 |
| `--strategy` | `rb` | `rb` / `sb` / `hero` / `dynamics` |
| `--show-ideal` | false | 叠加仿真理想角点（仅调试） |
| `--probe-image` | 空 | 只对单张图跑检测后退出，例如 `--probe-image=/tmp/frame.png` |
| `--wait` | 8 | 等待仿真启动的秒数 |

`--fire-tolerance` 的作用：仿真的云台是带速度/加速度限制的二阶伺服，实际角会滞后于规划角。Planner 的 `plan.fire` 只看规划角，直接照它开火会在云台还没跟上时打偏。本适配器额外要求：

```text
|actual_yaw - plan.yaw| <= fire_tolerance
|actual_pitch - (-plan.pitch)| <= fire_tolerance
```

静止靶上这个门限几乎不影响开火；平移/自转靶上它会减少无效弹、提高命中率。日志里的 `align=(yaw_err, pitch_err)°` 就是这两个量。

## 6. 配置要点

`configs/rm_aim_sim.yaml` 以 `rb_auto_aim.yaml` 为骨架，改成了仿真相机参数：

```yaml
enemy_color: "blue"          # 仅 --color-filter 时生效
yolo_name: yolov8
yolov8_model_path: ../assets/yolov8.xml
device: CPU
min_confidence: 0.30

camera_matrix: [1303.6753236814714, 0, 720, 0, 1303.6753236814714, 540, 0, 0, 1]
distort_coeffs: [0, 0, 0, 0, 0]
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
R_camera2gimbal: [0, 0, 1, -1, 0, 0, 0, -1, 0]
t_camera2gimbal: [0.08, 0, 0.04]

gimbal_delay: 0.002
gimbal_control_delay: 0.04
high_speed_delay_time: 0.030
low_speed_delay_time: 0.015
```

- `camera_matrix` / `R_camera2gimbal` / `t_camera2gimbal` 只是给 `Solver` 初始化用；每帧 `frame.json` 会覆盖，所以改仿真分辨率/FOV 不用改这里。
- `R_gimbal2imubody` 用单位阵即可：适配器走 `set_R_gimbal2world_from_tf`，不走 IMU 中间系。
- `uv_observation`、`multi_armor_fusion` 第一版都关掉，先把“检测 → PnP → EKF → MPC”跑通。仿真单靶场景打开多板融合收益有限。
- `auto_fire` 字段本程序不读；是否开火由 `--fire` 决定。

## 7. 验收顺序

按仿真仓库建议，逐级验证，不要一上来就追求命中率。

### 7.1 通信：固定角、不开火

```bash
cd ~/Desktop/sp_vision_25_rbclone/build
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --duration=5
```

看日志：

```text
[control] yaw=... pitch=...(下发,下正) fire=false align=(...)°
```

以及仿真界面/`state.json` 的 `gimbal.yaw/pitch` 是否靠近规划值。`state.ipc.last_seq` 应在增长，`rejected` 不持续增加。

### 7.2 相机与检测

```bash
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --no-display --duration=5
```

应看到 `[perception] ... armors=1 target=yes tracker=tracking`。如果 `armors=0`：

- 确认仿真不是 `--headless`，窗口在渲染；
- 确认 `--blur=3`（默认）；
- 靶车在 4 m、默认曝光 0.4 下应该能检出；更远/更暗可把仿真“相机曝光增益”调到 1.0。

只想验证单帧检测、不连仿真：

```bash
# 先用仿真界面的截图/相机 PNG 导出（captures/）得到一张仿真画面，再：
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --detector=traditional \
  --probe-image=/tmp/frame.png
```

### 7.3 PnP / 真值对比

```bash
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --show-ideal
```

灰线是仿真理想角点，只用于肉眼对比；控制不读它。静止靶下检测框应和灰线基本重合，日志的 `target=(x,y,z)` 应接近 `truth.armors` 中的装甲中心（云台系，z 减 0.65）。

### 7.4 静止靶

默认场景（4 m、静止）开火：

```bash
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --fire --duration=12
```

本机实测（Linux x86_64，OpenVINO CPU，traditional+blur3，`--fire-tolerance=1.0`）：

```text
fired=24 resolved=24 hit=24 ground=0 expired=0 hit_rate=100.0% damage=350 kills=1
```

### 7.5 平移靶

仿真仓库自带 `configs/translate.cfg`（1.5 m/s、行程 ±1.5 m、加速度 3 m/s²）。本机实测约 **18/40 = 45%**，11 s 击毁。平移换向时 EKF 速度估计会滞后，命中率明显低于静止靶，属于正常的算法调参范围。

### 7.6 自转靶

```bash
# 终端一：把下面内容保存为 /tmp/sim_spin_h0.cfg
cd ~/Desktop/rm_aim_sim
./run.sh -- --remote --config=/tmp/sim_spin_h0.cfg

# 终端二
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --fire --duration=15
```

`/tmp/sim_spin_h0.cfg`：

```ini
[target]
rotate=true
omega=4.0
height_difference=0.0

[camera]
brightness_gain=1.0
```

实测 **22/29 = 75.9%**，约 10 s 击毁。

> **重要**：仿真默认 `height_difference=-0.1`（两组装甲下沿差 100 mm）。本仓库标准 `Target` 模型按四块装甲同高处理，自转时奇偶装甲交替出现会造成约 ±0.05 m 的 pitch 偏差；同一组参数在默认高度差下只有 **18/57 ≈ 32%**。做算法对比时建议先把仿真的 `height_difference` 设为 0；要测高度差场景，需要算法侧显式估计每块装甲的高度，不能拿本适配器当“已支持”。

### 7.7 组合靶

```bash
# 终端一：仿真自带组合场景
cd ~/Desktop/rm_aim_sim
./run.sh -- --remote --config=res://configs/combined.cfg

# 终端二
./rm_aim_sim_client ../configs/rm_aim_sim.yaml --fire --duration=20
```

组合场景同时有平移和自转，命中率通常低于单项；先用不带 `--fire` 的模式看跟踪是否稳定，再开火。`combined.cfg` 也带默认 `height_difference=-0.1`，若要和 7.6 一样排除高度差，复制一份并加 `height_difference=0.0`。

## 8. 调参建议

按影响从大到小：

1. **仿真的角分辨率/曝光/距离/高度差**：先保证 `armors=1` 且跟踪稳定。能改仿真就把相机提到 2560×1920 + `--blur=0`；只能用 1440×1080 时用 `--blur=3`；更暗就调曝光。
2. **`--fire-tolerance`**：默认 1.0°。云台伺服慢、目标快时，适当放宽到 1.5–2° 可以提高开火频率，但会牺牲单发精度；收紧到 0.5° 则更挑时机。
3. **`gimbal_delay` / `high_speed_delay_time` / `low_speed_delay_time`**：
   - `gimbal_delay` 加进 Planner 的目标预测时长，补偿“发弹命令 → 出膛”之外的延迟；
   - 自转 > `decision_speed`（默认 8 rad/s）用 `high_speed_delay_time`，否则用 `low_speed_delay_time`；
   - 先保持默认，按平移/自转的命中分布微调，不要一次改多个。
4. **`yaw_offset` / `pitch_offset`**：静态靶中心偏移用这两个（度）；本机静态靶 24/24，不需要额外偏置；若你的模型/PnP 有系统偏差再调。
5. **`target_dist_error` / `target_h_error`**：弹道距离/高度补偿（m）。枪口在云台前 0.22 m，必要时用 `target_dist_error` 做小修正。
6. **检测器**：`--detector=yolo` + 仿真域模型；或继续用 traditional。`--blur` 先保持 3，确认检测稳定后再尝试 0/5。

调参时看日志里的：

```text
target=(x,y,z) yaw=... vyaw=...
align=(yaw_err,pitch_err)°
sim fired/hit/ground/expired
```

`ground` 多通常是 yaw 偏（弹丸越过目标后落地）或 pitch 偏低；`expired` 多通常是 yaw 提前量/相位不对。两者要分开看。

## 9. 故障排查

| 现象 | 检查 |
|---|---|
| `等待 .../rm_aim_sim_default 超时` | 仿真没启动、channel 不一致、仿真用了 `--no-ipc` |
| `同一通道已有其他客户端占用 client.lock` | 退出 `camera_demo.py` / `control_demo.py` / 另一个自瞄 |
| `等待图像帧超时` | 仿真 `--headless`、窗口最小化/无渲染、GPU/OpenGL 问题 |
| 有图但 `armors=0` | `--blur=3` 是否生效；曝光/距离；试 `--detector=yolo` 或 `--probe-image` 单帧排查 |
| 云台不动 | 仿真是否 `--remote`/勾选“本机自瞄控制”；`state.ipc.enabled`、`paused`；看 `last_seq/rejected` |
| 云台反着动/上下颠倒 | 确认 `pitch_sim=-plan.pitch` 没被改回；检查 `frame.json` 的 `R_camera2gimbal` 与你的配置是否一致 |
| 自转/平移命中低 | 先看 `align`；再检查仿真 `height_difference`；最后调 `gimbal_delay`/延迟 |
| 红蓝颠倒/识别错色 | RGB→BGR 已处理；`--color-filter` 默认关闭；`enemy_color` 只在开启过滤时生效 |
| 目标死亡后不再检测 | 正常：仿真血量归零会熄灭灯条，`camera.armors` 也清空；按 R 重置或重启仿真 |

## 10. 已知边界

- 本适配器只做“接口 + 现有算法”的闭环，不改进检测模型、跟踪器或火控；仿真命中率不等于实车命中率。
- 仿真没有车体碰撞、完整遮挡、散布、相机噪声/运动模糊；空气阻力是水平恒减速简化模型（`[weapon] drag_k/drag_m`，默认与 `TrajectoryV2` 的 17 mm 小弹丸一致），弹道和云台仍是简化模型。
- 仿真从 `barrel_length=0.22 m` 的枪口出膛，而本仓库 `Planner::rbaim` 从云台原点解算弹道；5 m 处这会带来约 2–3 cm 的偏高水平，可用 `pitch_offset` 或后续的枪口偏移补偿处理。
- 仿真世界原点在地面，本仓库 `Target` 世界原点是云台；本适配器按云台系解算，不额外加 `gimbal.origin`。
- `--show-ideal` 和日志里的 `truth_angle`/`truth` 只用于调试评分，**不能**作为控制输入；否则测出来的不是图像识别效果。
- 同一通道只允许一个客户端；多场景并行时改仿真的 `[ipc].channel` 和本程序的 `--channel`。
- `planner.hpp` / `planner.cpp` 里两处逐帧日志从 `warn/info` 降到了 `debug`，避免 100 Hz 控制循环刷屏；文件日志仍保留 debug。这是本适配器的配套改动，不影响算法逻辑。

## 11. 能量机关（打符）接入：`rm_aim_sim_rune_client`

当前版本的仿真支持 `[target].mode="rune"`：五个靶面绕中心旋转、小符亮 1 个 / 大符亮 2 个、2.5 s 主击窗口、1 s 第二击窗口、激活环数、失败重试，以及大符激活期间的正弦变速。它和装甲模式共用相机、云台、弹道、共享内存和统计，只是把目标换成机关。

打符侧入口是 `auto_buff::RuneSystem`（`tasks/auto_buff/rune_system.hpp`）。`src/rm_aim_sim_rune_client.cpp` 复用同一套共享内存协议，把“相机输入 + 云台通信”替换成仿真，算法仍走本仓库的检测/UV 位姿/相位拟合/弹道火控。

### 11.1 新增文件

```text
src/rm_aim_sim_rune_client.cpp   共享内存客户端 + RuneSystem 感知线程 + 100 Hz 控制线程
configs/rm_aim_sim_rune.yaml     打符专用机器人配置（仿真默认 45° 相机）
configs/rm_aim_sim_rune_real.yaml 实车对齐场景配置（FOV 33.17°/fx=1813.13）
CMakeLists.txt                   rm_aim_sim_rune_client 目标
```

### 11.2 构建与启动

```bash
# 终端一：仿真进入能量机关模式（6 m 小符，默认 60°/s）
cd ~/Desktop/rm_aim_sim
# 推荐：实车对齐弹道/相机（三维阻力+Magnus、枪口 0.0988 m、FOV 33.17°）
./run.sh -- --remote --config=res://configs/rune_real.cfg
# 大符换 rune_real_large.cfg；默认 45° 场景用 rune.cfg / rune_large.cfg

# 终端二：本仓库
cd ~/Desktop/sp_vision_25_rbclone
cmake --build build --target rm_aim_sim_rune_client -j8
cd build
./rm_aim_sim_rune_client ../configs/rm_aim_sim_rune_real.yaml          # 只瞄准
./rm_aim_sim_rune_client ../configs/rm_aim_sim_rune_real.yaml --fire   # 允许开火
```

默认 45° 场景把两处 `rune_real` 换成 `rune` 即可，**内参必须和仿真相机一致**：`rune.cfg` 用 `rm_aim_sim_rune.yaml`（fx=1303.7），`rune_real*.cfg` 用 `rm_aim_sim_rune_real.yaml`（fx=1813.13）。

运行目录必须是 `build/`（模型路径按仓库根目录解析）。同一通道只能有一个客户端：先退出装甲客户端、`camera_demo.py`、`control_demo.py`。

也可以不启动仿真，先验证模型对某张仿真截图的输出：

```bash
./rm_aim_sim_rune_client ../configs/rm_aim_sim_rune.yaml \
  --probe-image=/path/to/frame.png --rune=small --color=blue
```

### 11.3 数据流与五个必须遵守的约定

```text
request.json ──> 仿真渲染一帧
frame.json + frame-*.raw (1440x1080 RGB/RGBA)
        │
        ├─ gimbal.quaternion_wxyz ──> RuneSystem 的 imu_q（本仓库 R_gimbal2imubody=单位阵）
        ├─ camera.K / R_camera2gimbal / t_camera2gimbal ──> 只用于校验，见下
        └─ 图像 ──> RuneDetector 五点 ──> 轮廓精修/UV 位姿 ──> 相位拟合 ──> 弹道火控
                                                                  │
command.json（100 Hz，yaw/pitch/fire）<── RuneCommand（弧度、抬头 pitch 为正）
```

1. **pitch 符号**：`RuneCommand.pitch` 抬头为正，仿真协议 pitch 向下为正，下发时必须 `pitch_sim = -command.pitch`；yaw 同号。写反会看到云台朝反方向抬。
2. **时间戳必须是 `steady_clock`**：`RuneDecisionModule` 用 `function::getNowTimestamp()`（即 `steady_clock::now()`）与每帧 `capture_time` 比较，判断 `temp.data_life`、`critical_timeout`、`max_life_time`。如果直接把仿真的 `sim_time`（从 0 开始的小数秒）传进去，所有帧都会被判成“已经过期十几万秒”，表现为 `found` 永远出不来。客户端用“收到帧的时刻 − `capture_timing` 的 publish/readback 开销”作为 `capture_time`；这也是打符和装甲适配最大的区别之一。
3. **内参只读一次**：`RuneSystem` 在构造时从 YAML 读 `camera_matrix`/`distort_coeffs`/`R_camera2gimbal`/`t_camera2gimbal`，之后**不会**随帧更新（装甲侧的 `Solver` 有逐帧覆盖接口，打符侧没有）。所以 `configs/rm_aim_sim_rune.yaml` 里的内参必须和仿真当前相机一致；启动时客户端会对比 `frame.json` 并打印告警。仿真默认 1440×1080 / 垂直 FOV 45° 对应 `fx=fy=1303.6753`（`rm_aim_sim_rune.yaml`）；实车对齐场景 FOV 33.17° 对应 `fx=fy=1813.1276`（`rm_aim_sim_rune_real.yaml`）。
4. **颜色与大小符**：默认 `--color=auto` 用仿真 `appearance.team`（机关靶面按靶方颜色渲染），`--rune=auto` 用 `state.rune.kind`。手动指定用 `--color=red|blue`、`--rune=small|big`；切换时 `RuneSystem` 会自动 `reset()`。
5. **失效保护**：仿真侧 `ipc.enabled=false`、`paused=true`、`state.json` 超过 200 ms 未更新，或目标超过 `--target-timeout-ms`（默认 300 ms）都会停止下发控制；丢帧后最多保持最后瞄准角 `--hold-ms`（默认 150 ms），保持期间不开火。客户端退出时会尽力发一包 `control=false, fire=false`。

### 11.4 命令行选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `config.yaml`（位置参数） | `../configs/rm_aim_sim_rune.yaml` | 机器人 YAML |
| `--channel` | `default` | 必须与仿真 `[ipc].channel` 一致 |
| `--rune` | `auto` | `auto` / `small` / `big`；auto 读 `state.rune.kind` |
| `--color` | `auto` | `auto` / `red` / `blue`；auto 读 `appearance.team` |
| `--fire` | false | 允许开火；不加只瞄准 |
| `--no-display` | false | 不开调试窗口（仿真仍须图形模式） |
| `--duration` | 0 | 运行秒数，0 为一直运行 |
| `--fire-tolerance` | 1.0 | 实际云台与下发角的开火对齐容差（度） |
| `--hold-ms` | 150 | 丢目标后保持最后瞄准角的时间 |
| `--target-timeout-ms` | 300 | 目标过期时间 |
| `--probe-image` | 空 | 只对单张图离线跑检测后退出 |
| `--wait` | 8 | 等待仿真启动的秒数 |

### 11.5 验收顺序

1. **模式与图像**：仿真左侧“目标模式”必须是“能量机关模式”，状态栏能看到 `正在激活` 和亮的靶面；`./rm_aim_sim_rune_client ... --no-display --duration=5` 的日志里 `[rune] dets` 不应恒为 0。只有 `camera.armors` 输出点亮靶面，`truth.armors` 始终是 5 个。
2. **检测/位姿**：`[rune]` 行里 `dets=1`、`fail=0`、`uv=1`（或 `pnp=1`）说明网络 + 位姿链路通了。`fail=5` 是 `DETECTION_EMPTY`，`fail=6` 是 `CORE_NOT_READY`（还没形成稳定符目标）。
3. **相位与瞄准**：日志出现绿色 `[update_candidate_targets]顺时针/逆时针` 后，`[control]` 行的 yaw/pitch 应该跟着点亮靶面转（6 m 小符：yaw 约 ±7°，pitch 约 −16°…−29°），`fresh=true`。
4. **开火**：先 `--fire-tolerance=2.0` 跑，确认 `fire=true` 时 `state.json` 的 `stats.fired` 增长；再收紧容差。仿真没有血量，命中只推进 `rune_rings`（小符 5 环、大符 10 环），失败会按 `rune_failures` 计数并自动重试。
5. **结束汇总**：客户端退出时打印 `尝试/成功/失败/环数` 与仿真 `fired/hit/rate`。

### 11.6 本机实测（Linux x86_64、OpenVINO CPU、1440×1080）

在默认 6 m 小符场景抓 150 帧（约 23 fps，其中 92 帧有亮靶面）离线回放：

| 场景 | 亮靶面帧 | 有检测的帧 | 说明 |
|---|---:|---:|---|
| 6 m，FOV 45°，增益 0.4（默认） | 92 | 66（72%） | 检测断续，间隔可达数帧 |
| 6 m，阈值 0.65 → 0.30 | 92 | 85（92%） | 降低置信度可补一部分，但会引入误检 |
| 4 m，FOV 33°，增益 1.0 | 72 | 22（31%） | 靶面更大反而更差，且 4 m 时 pitch 接近 −30° 上限 |

实机闭环（`--fire --fire-tolerance=1.5`，同一份 `rm_aim_sim_rune_client`）：

| 仿真场景 | 时长 | 发射 / 命中 | 激活 | 环数 |
|---|---:|---|---:|---:|
| 默认 `rune.cfg` + 旧 `horizontal` 弹道 | 30 s | 11 / 0 | 0 | 0 |
| `rune_real.cfg`：quadratic + Magnus、0.0988 m 枪口、FOV 33.17° | 30 s | 14 / 14（100%） | 2 | 14 |
| 同上 | 60 s | 40 / 40（100%） | 3 | 37 |

旧弹道的 11 发不是随机散布：把 `state.events` 的出膛角和当时的仿真弹道反算到机关平面（x=6 m），弹着点稳定偏高 **0.32–0.51 m**，而靶面半径只有 0.15 m，所以全脱。原因是仿真默认的 `drag_model="horizontal"`（水平恒减速、无 Magnus、枪口 0.22 m）和打符火控的三维二次阻力 + Magnus（`gune_length=0.0988`）不是同一套物理。仿真改成 `drag_model="quadratic"`、`quadratic_drag=0.019366`、`magnus_acceleration=-0.008`、`barrel_length=0.0988` 后，同一份算法 **100% 命中**。

**结论：之前的 0 命中主要是仿真弹道模型不真实，不是打符算法本身。** 检测仍有约 70% 的单帧检测率（大符约 44%），会让一部分激活窗口打不完（60 s 里 16 次尝试完成 3 次），但**只要开火就能命中**。检测层的根因和 2026-10-28 的仿真外观修正见 §11.9，弹道层的修正见 §11.10。

两个仍然成立的测试约定：

- 不要用“把符调慢”来 isolate 问题：小符滤波器在 `reset()` 时把角速度固定成 `small_phase_estimate.expect_angular_velocity=1.0472 rad/s`（官方 60°/s）。把仿真改成 0.15 rad/s 会让预测相位持续滞后（实测慢速场景只有 1/9 命中、弹着点沿切向偏），这不是算法在 60°/s 下的真实表现。
- 不要用旧 `horizontal` 场景的命中率评价打符算法，也不要给旧场景标 `pitch_offset`；换 `rune_real*.cfg` 即可。

### 11.7 想让命中率上来，按这个顺序改

1. **补检测连续性（收益最大，已在 §11.9 定位并修在仿真侧）**：根因不是单纯的颜色阈值，而是仿真缺了规则图 5-19/5-20 的挑战臂实心灯条和流水/靶面自发光。仿真修好后 180 帧离线回放从 `detection 67 / found 0` 提升到 `detection 180 / found 151`。如果换回未修改的仿真外观，可选：
   - 用仿真 `state.camera.armors` 的四个角点 + `state.truth` 自动标注仿真截图，微调 `model-0624.onnx`（这是最根本的做法）；
   - 或改仿真的符材质/曝光，让靶面更接近真实自发光外观，再重新评估；
   - 临时降低 `power_rune.detector.confidence_threshold`（0.3 左右）只能补一部分，且会引入误检。
2. **放宽相位滤波**（仿真帧率 20–30 fps、检测有空洞时）：`configs/rm_aim_sim_rune.yaml` 已经默认设置
   ```yaml
   power_rune:
     small_phase_estimate:
       max_data_interval: 0.25          # 实车默认 0.1
       min_size_to_confirm_rotation: 10 # 实车默认 20
   ```
   这只是让滤波器容忍丢帧，不改善检测本身；换回实车配置时请删掉 `power_rune` 节点。
3. **弹道对齐（已于 2026-10-29 修在仿真侧）**：用 `configs/rune_real.cfg` / `rune_real_large.cfg` + `configs/rm_aim_sim_rune_real.yaml`（fx=1813.13）。仿真新增 `drag_model="quadratic"`、`quadratic_drag`、`magnus_acceleration`，与 `PowerRuneBallisticModel` 同形；同时把枪口长度设成实车配置的 0.0988 m。不要再给旧 `horizontal` 场景标 `pitch_offset`（例如 −4.6°）来“修”弹道，那只是补偿一种不真实的物理模型，换距离就失效，更不能带回实车。
4. **别用非官方转速测小符**：小符滤波器固定期望 1.0472 rad/s（60°/s）；`rune_speed` 改成 0.15/0.5 等值会引入相位滞后，不能用来判断算法好坏。大符在激活期间本身是正弦变速，适合测运动拟合，但外观和检测同样受第 1 条限制。
5. **别把机关放太近**：`center.z=2.45 m`、云台 pitch 上限 −30° 时，4 m 已经需要约 −24° 基础俯仰，加上弹道补偿会顶到限位；默认 6 m 是可用范围。

### 11.8 已知边界

- 仿真只输出点亮靶面的理想角点；未点亮靶面在图像里是暗的，不能拿 `truth.armors` 当检测输入。
- 仿真不模拟裁判系统激活时机、红蓝共轴联动、增益结算和官方 500 ms 速度误差；自动激活循环只用于调试。
- 打符火控的 `is_enable_fire` 是算法自己的判据，本适配器只额外加“实际云台对齐”门限；两者都为真才开火。
- 仿真命中率不等于实车命中率；先用 `auto_buff_test` + 真实录制的符视频验证检测/位姿，再用本适配器验证控制闭环。

### 11.9 检测断续的根因与仿真外观修正（2026-10-28）

逐项隔离后，检测断续的主因不是颜色阈值或曝光参数，而是仿真缺了规则图 5-19/5-20 的两处灯效：

1. **被挑战的灯臂中轴实心灯条没亮**。仿真原来只在命中后才显示 `Arm_*_BarOn`，挑战中只有流水箭头；而 `Projector::is_valid_for_projection()` 要求 `armor_module` / `light_arm` / `center_R` 三个语义轮廓全部可用，点状流水形成的轮廓太小，`light_arm` 基本不成立，所以即使五点检测正常，`found` 也出不来。逐项回放验证：隐藏流水 → 检测 0/24；保留流水但隐藏靶面 → 检测 0/24；只亮靶面 → 0/24；亮整条灯臂后，`found` 才出现。
2. **挑战靶面/流水不是 LED 自发光**。CAD 的 `Standby` 靶面是 `Plate*`（无 emission），流水 shader 的 `energy` 只有 1.7；默认 0.4× 曝光下流水是一条暗点线。把流水提亮后单帧检测从 11/24 升到 20–24/24，但 `found` 仍为 0；再点亮挑战臂灯条并给靶面加最外圈后，180 帧连续旋转回放达到 `detection_frames=180, found_frames=151`。

仿真侧已修改（`rm_aim_sim/scripts/world_view.gd`，只影响渲染，不改命中判定）：

- 挑战臂亮 `bar_on`：小符整条；大符按 1/5…5/5 进度，第一组至少 1/5；
- `Plate*` 靶面改成 unshaded 自发光；流水 shader 使用 `RUNE_FLOW_ENERGY`；
- 挑战靶显示最外圈 `Hit_1`（图 5-19/5-20 的靶面外环）。

仍然存在的非颜色问题：

- **斜向相位仍偏弱**：45°/135°/225° 附近的单帧置信度低于正向，最差时只有 0.0–0.1。根因是网络输入固定 640×480，6 m 时靶面只有约 28 px，斜向时点状流水和 4 个关键点更接近；灯臂亮起后检出帧率已接近 100%，但斜向的 `kp` 精度不如正向。彻底解决要微调模型或提高仿真相机/目标的有效尺寸，单纯降阈值会引入误检。
- **大符首先卡在位姿，而不是 100 帧运动模型**：大符第一组只亮 1/5 灯臂，`light_arm` 轮廓太短，`armor_module/light_arm/center_R` 三件套不成立，日志里是 `pnp=0 uv=0`、`found=false`。把仿真挑战臂临时改成整条（仅诊断）后位姿立刻正常：默认 100 帧门槛下 6 发 2 中，放宽到 20 帧后 9 发 8 中、完成 1 次激活。也就是说大符的控制/弹道本身能工作，但“首次挑战的 1/5 灯臂”是算法在真实大符上也会遇到的位姿弱区，需要用真实大符首组视频确认；`big_phase_motion_estimate.min_data_size_to_build_motion_model=100` 是第二道门槛，仿真里可以临时放宽。
- **开火不中是另一层问题**：检测修好后 `fired` 会增加，但 6 m 处仍有 §11.6 的约 −4.6° 系统性仰角偏差；这是火控弹道模型与仿真弹道的差异，不是识别断续。

复现命令（离线回放最直观）：

```bash
# 仿真侧渲染 180 帧连续旋转（60°/s，30 fps）后：
./build/auto_buff_test --headless=true --config-path=../configs/rm_aim_sim_rune.yaml \
  --mode=small --color=blue /tmp/rune_fixed.mp4
# 修改前：detection_frames=67  found_frames=0
# 修改后：detection_frames=180 found_frames=151
```


### 11.10 弹道对齐：从 0 命中到 100%（2026-10-29）

仿真原来的默认弹道是 `drag_model="horizontal"`：`a = drag_k*v0²/drag_m` 只作用在水平方向，垂直只有重力，枪口 0.22 m。它刻意匹配装甲自瞄的 `tools::TrajectoryV2`，但不是完整的三维弹道。打符火控 `PowerRuneBallisticModel` 用的是三维二次阻力 + Magnus（`drag_coefficient=0.42`、`air_density=1.3`、`reference_area=0.000227`、`small_bullet_mass=0.0032` → `k=0.019366`，`magnus=-0.008`，`gune_length=0.0988`）。两者在 6 m、23 m/s 下垂直弹道差约 0.3–0.4 m，正好让 0.15 m 半径的靶面全脱。

仿真侧新增（`scripts/settings.gd`、`scripts/simulation.gd`）：

- `[weapon] drag_model = "horizontal" | "quadratic"`；
- `quadratic_drag`（1/m）和 `magnus_acceleration`；
- quadratic 模式用 1 ms 步长 RK4 积分三维 `a = -k|v|v + Magnus + gravity`，弹道表按需生成，碰撞/拖尾/命中查询用同一张表；
- `configs/rune_real.cfg` / `rune_real_large.cfg` 固定 `quadratic`、`k=0.019366`、`m=-0.008`、`g=9.7887`、`barrel_length=0.0988`、`vertical_fov=33.17`。

验证（同一份 `rm_aim_sim_rune_client`、同一相机/枪口，只换弹道）：

| 弹道 | 发射 | 命中 | 命中率 | 环数 |
|---|---:|---:|---:|---:|
| horizontal | 11 | 0 | 0% | 0 |
| quadratic + Magnus | 14 / 40 | 14 / 40 | 100% | 14 / 37 |

复现：

```bash
cd ~/Desktop/rm_aim_sim
./run.sh -- --remote --config=res://configs/rune_real.cfg
cd ~/Desktop/sp_vision_25_rbclone/build
./rm_aim_sim_rune_client ../configs/rm_aim_sim_rune_real.yaml --fire --fire-tolerance=1.5
```

注意：`rune_real*.cfg` 把相机改成 FOV 33.17°（fx=1813.13），客户端必须用 `rm_aim_sim_rune_real.yaml`；用默认 `rm_aim_sim_rune.yaml`（fx=1303.7）会触发内参不一致告警并算错位姿。仿真的 `drag_model` 默认仍是 `horizontal`，装甲自瞄的 `TrajectoryV2` 验收不受影响；两套命中率不能混比。
