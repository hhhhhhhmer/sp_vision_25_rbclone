# 阶段 2~4：UV（像素/UVL）观测改造完成报告

> 承接 `docs/uv_vs_pnp_assessment.md`（评估）与 `docs/multi_armor_step1_results.md`（步骤 0+1 基线）。

## 1. 一句话结论

UV 观测已经完整落地并跑通阶段 2/3/4：**逐帧不再做 PnP + 140 步 yaw 搜索**（PnP 只用于首帧初始化），
多装甲联合批量更新、可见性/关联、单装甲退化保护、前哨/平衡/基地与长短焦切换全部支持。
快/ 全部 7 段录像离线 A/B：**一步预测误差中位数平均 14.1px，优于调好的 ypd 基线 14.6px**（7 段中 5 段更优），
半径/旋转相位稳定性大幅提升（半径四分位距 0.024~0.069 vs 0.068~0.199）。
代价是**深度抖动偏大**（z 二阶差分 12~63 vs 7~25），且 **UV 对相机标定精度更敏感**（见第 6 节风险）。

### 1.1 附：`meas_*` 与 `uv_*` 是两条互不相干的噪声链

| 模式 | 观测来自 | R 矩阵来自 | `meas_*` 是否生效 |
|---|---|---|---|
| 传统 YPD | `RVfromFYT::prepare_measurement()` 压出的 4 维 `[yaw,pitch,distance,armor_yaw]` | `meas_azimuth/distance/angle_sigma`（缺省时用原启发式） | ✅ |
| UV | 检测角点直接算出的 8 维 UVL | `uv_sigma_px / uv_sigma_len_ratio / uv_sigma_attitude_deg / uv_attitude_common_ratio / uv_sigma_angle` | ❌ |

`meas_*` 只在 `prepare_measurement()` 里被读（`rv_from_fyt.cpp:136-144`），而 UV 模式下
`Target::update()` 在 `if (ekf_.uv_ready())` 处提前 return（`target.cpp:241`），**根本走不到那个调用**。
实测：UV 配置把 `meas_*` 设成 9.0（荒谬地松）或 0.0001（荒谬地紧），与完全不设**逐帧 CSV 完全相同**。

## 2. 代码结构

| 文件 | 作用 |
|---|---|
| `tasks/auto_aim/tracking/uv_model.hpp/.cpp` | **新增**：UVL 观测模型（前向 h、解析雅可比 H、噪声模型、可见性、角点投影） |
| `tasks/auto_aim/model/camera_geometry.hpp` | **新增**：`CameraGeometry`（内参/畸变/相机→云台/云台→世界），UV 观测所需的几何载体 |
| `tasks/auto_aim/model/armor_interfaces.hpp` | `IArmorPoseSolver` 新增 `camera_geometry()` 虚函数（默认 unsupported） |
| `tasks/auto_aim/geometry/solver.cpp` | 实现 `camera_geometry()`；**Solver 保留**用于首帧初始化、重投影调试、前哨 pitch 判别 |
| `tasks/auto_aim/tracking/rv_from_fyt.hpp/.cpp` | UV 模式：`enable_uv` / `set_camera_geometry` / `associate_uv_armor` / `add_uv_observation` / `correct_uv_batch`；深度过程噪声可按模式缩放 |
| `tasks/auto_aim/tracking/target.hpp/.cpp` | `update_batch()`（多装甲联合更新）、`set_uv_config()`、`bookkeep_after_update()` 统一记账 |
| `tasks/auto_aim/tracking/tracker.cpp/.hpp` | UV 配置解析与逐帧注入、UV 分支（多装甲批量 / 前哨基地单块顺序）、几何不可用时自动回退传统路径 |
| `tests/uv_jacobian_test.cpp` | **新增**：前向模型 vs OpenCV、解析雅可比 vs 中心差分、噪声模型、退化输入 |
| `tests/uv_filter_test.cpp` | **新增**：端到端收敛测试（1/2/4 块装甲可见），量化多装甲融合与单装甲退化保护效果 |

### 2.1 观测定义（8 维 UVL）

每条灯条 4 维：`[角度, 中心x, 中心y, 长度]`，一块装甲板两条灯条共 8 维。

- 角度 = `atan2(dx, -dy)`（竖直灯条为 0，避开 `atan2(dx,dy)` 在 ±π 的回绕）
- 中心 = 上下端点均值（px），长度 = `|上-下|`（px）
- 观测量直接由检测到的 4 个角点构造（`Armor::points` 顺序 `[左上,右上,右下,左下]` 正好是两条灯条的端点），**不需要 PnP**

### 2.2 噪声模型（关键设计）

```
端点噪声      σ_endpoint = sqrt(σ_px² + (k_len·len_px)²)      # 检测噪声 + 尺度不变项
灯条中心      σ_center   = sqrt(σ_endpoint²/2 + σ_ind²)        # σ_ind 来自姿态误差的独立部分
灯条长度      σ_length   = σ_endpoint·√2                       # 差分量，不受共模平移影响
灯条角度      σ_angle    = sqrt(σ_angle0² + (σ_endpoint·√2/len_px)²)
姿态误差共模项 R += σ_com² · J_att J_attᵀ                      # J_att = ∂h/∂(云台姿态误差), 8x3
其中 σ_com = ratio·σ_att, σ_ind = sqrt(1-ratio²)·σ_att
```

设计要点：
1. **长度是差分量**，对云台姿态误差（整帧共模平移）不敏感，是深度信息的主要来源；
2. **姿态误差按比例拆成共模 + 独立两部分**：完全共模（ratio=1）会让滤波器无法利用"共同平移"里的方位信息
   （实测大幅变差），完全独立（ratio=0）会过度自信；`uv_attitude_common_ratio` 暴露这个折中；
3. 共模项用 rank-3 相关矩阵表达而不是对角矩阵，多灯条/多装甲时不会被错误地"平均掉"。

### 2.3 阶段 3：多装甲联合更新 + 关联 + 退化保护

- **联合批量更新**：一帧内所有关联成功的装甲板拼成一个 8N 维观测，做一次 EKF 更新
  （`Target::update_batch` → `RVfromFYT::correct_uv_batch`），而不是逐块顺序更新；
- **关联**：可见性（装甲板法线与视线夹角余弦 `uv_facing_cos_min`）→ 自适应像素粗门限
  （`max(配置值, 3σ_预测)`，状态散开时自动放宽）→ UVL 马氏距离精门限 → 同帧 ID 不重复；
- **单装甲退化保护**：`uv_radius_prior_sigma` 收紧半径先验（中心深度↔半径是弱可观测方向）。
  端到端测试（`uv_filter_test`）量化效果：

  | 每帧可见装甲 | 位置误差 | 半径误差 | 半径估计 |
  |---|---|---|---|
  | 1 块 | 0.110 m | 0.046 m | 0.246（真值 0.200） |
  | **2 块** | **0.0022 m** | **0.0026 m** | **0.197** |
  | 4 块 | 0.053 m | 0.000 m | 0.200 |

  同一测试里若不收紧半径先验（P0[8]=1），半径会漂到 0.55、位置误差 0.38m —— 保护是必要的。

### 2.4 阶段 4：特殊目标与相机切换

- **前哨站（3 装甲 + 高度阶梯）/ 基地（3 装甲）/ 平衡（2 装甲）**：UV 模型复用 `armor_num` 与
  `h_armor_xyz` 的高度/半径约定；前哨站高度锚点由 `bookkeep_after_update` 用**预测**的装甲高度累加，
  再喂给 `tower_height_multiplier`；Tracker 中这三类目标**每帧只更新最靠近图像中心的一块装甲板**，
  不参与联合批量（见 §9 修订记录：一帧内顺序更新多块会把高度锚点的采样窗口压到 1~2 帧）；
- **高度阶梯只有一份实现**：`tower_armor_height_multiplier`（`tracking/tower_armor_geometry.hpp`）
  同时被 `UvModel::armor_position`（观测几何）与 `RVfromFYT::h_armor_xyz`（瞄准几何）调用，
  锚点无效时一律返回 0（视为与 0 号板同高）；禁止在任一侧另写一份；
- **长短焦切换**：`Tracker::apply_uv_config()` **每帧**从当前求解器取 `camera_geometry()`，
  而双目入口在切换时会 `tracker.setSolver(另一台相机的 solver)`，因此内参/外参会自动跟随切换；
- **回退路径**：求解器不提供几何（例如测试桩）时 `uv_ready()==false`，Tracker 自动退回传统 ypd 路径。

## 3. 离线 A/B（快/ 全部 7 段录像）

配置：`base`=现状；`t2m`=步骤 1 调好的 ypd 基线；`uv`/`uvq`=UV（`uvq` 把 `uv_process_noise_z_scale` 设为 0.1）。

### 3.1 一步预测误差中位数（px，越小越好）

| 录像 | base | t2m | **uv** |
|---|---|---|---|
| 2026-04-04_19-58-26 | 31.95 | 21.37 | **20.68** |
| 2026-04-04_19-59-07 | 32.80 | 20.76 | **17.87** |
| 2026-04-04_19-59-45 | 31.40 | **19.51** | 20.09 |
| 2026-04-04_20-01-41 | 39.90 | 17.72 | **17.23** |
| aim2（旋转） | 26.64 | 12.56 | **11.40** |
| blue2（旋转） | 12.10 | 7.36 | **6.35** |
| red1（旋转） | 8.35 | 3.19 | **2.74** |
| **7 段平均** | 26.2 | 14.6 | **13.8** |

> 注：uv 列已用**当前二进制**重跑刷新（早期版本漏编了一次 `rv_from_fyt.cpp`，那一列偏保守约 3%）。
> 旋转三段的数值受时间戳假设影响（见第 6 节第 6 条），只做定性参考。

### 3.2 其它指标

| 指标 | base | t2m | uv | 说明 |
|---|---|---|---|---|
| 跟踪连续性（平均 track%） | 97.8 | 98.6 | **99.1** | uv 中断次数 3/2/0/3/3/8/2 |
| 重投影误差（状态 vs 观测，均值 px） | 23~31 | 3~10 | **5~13** | 两者都远优于 base |
| 半径四分位距（越小越稳） | 0.076~0.230 | 0.068~0.199 | **0.002~0.069** | UV 明显更稳 |
| 深度抖动 z 二阶差分 | 6~24 | **7~25** | 12~63 | UV 偏大，`uvq` 可缓解 |
| 每帧前端耗时 | ~216 µs（PnP+140 步搜索） | ~216 µs | **~30-50 µs** | 省掉 140 步 yaw 搜索 |

> 说明：`uv` 与 `uvq` 只在深度过程噪声上有差别，`uvq` 用 ~3% 的预测精度换深度抖动下降 10~30%。

## 4. 单元测试

```
$ ./uv_jacobian_test
  [ ok ] 前向模型 vs cv::projectPoints（含畸变，4 块装甲，误差 < 1e-3 px）
  [ ok ] 解析雅可比 vs 中心差分（< 1e-4）
  [ ok ] 噪声矩阵正定 / 姿态误差对长度影响远小于中心 / ratio=0 独立、ratio=1 共模
  [ ok ] 灯条角度与长度对 yaw 的偏导非零（UVL 携带朝向信息）
  [ ok ] 倾角先验破坏 180° 对称性
  [ ok ] 可见性判据方向正确；2 点/NaN 观测被拒；相机后方投影返回零
$ ./uv_filter_test
  [ ok ] 1/2/4 块装甲可见时位置、深度、偏航、半径全部收敛
```

$ ./uv_regression_test
  [ ok ] 三种锚点状态下 UvModel 与 h_armor_xyz 的几何逐点一致
  [ ok ] 锚点无效时不凭空产生高度阶梯
  [ ok ] 静止匀速前哨站：瞄准 z 平均 0.046m / 最大 0.215m（同轨迹 YPD 0.067m / 0.250m）
  [ ok ] 高度锚点方向正确；update_count_ 同帧两次校正只计一次
```

`ctest`：20/20 通过（含新增 3 个 UV 测试与原有 kf/rune/safety 测试）；全项目所有可执行目标编译通过。

## 5. 上车建议

1. **先离线确认**：用 `tracker_offline_bench` 在本队录像上跑 `uv` / `uvq` / `t2m`，挑预测误差与深度抖动都满意的档位；
2. **配置**（`configs/rb_auto_aim.yaml`）：

   ```yaml
   uv_observation: true
   multi_armor_fusion: true
   uv_sigma_px: 1.0
   uv_sigma_len_ratio: 0.02
   uv_sigma_attitude_deg: 0.5
   uv_attitude_common_ratio: 0.0
   uv_sigma_angle: 0.1
   uv_associate_gate_px: 60.0
   uv_facing_cos_min: 0.15
   uv_radius_prior_sigma: 0.02
   uv_process_noise_z_scale: 1.0   # 深度抖动大就降到 0.1~0.01
   ```

3. **回退开关**：把 `uv_observation` 设为 false 即恢复传统 ypd 观测（步骤 1 的参数仍然有效），
   两条路径都保留、可随时对比；
4. **优先验证顺序**：小陀螺/横移目标下的瞄准点抖动 → 前哨站（高度阶梯）→ 长短焦切换 → 平衡/基地。

## 6. 已知风险与后续

1. **UV 对相机标定更敏感**：UV 的深度直接来自"灯箱像素尺度"，内参/畸变有偏差会直接变成深度偏差或抖动。
   1280×1024 那三段录像用的是 `configs/test/example.yaml` 的旧标定（`reproj self` 诊断 0.9~2.5px），
   UV 在这三段上预测误差均值（21/12/12px）明显差于中位数（12.8/6.5/2.7），说明存在少量大误差帧——
   **上车前应确认本车标定精度，必要时重新标定**。
2. **深度抖动**：UV 的深度由长度/尺度驱动，比 ypd 模式的"弱距离观测 + 模型外推"更活跃。
   已提供 `uv_process_noise_z_scale` 与 `uv_sigma_len_ratio` 两个旋钮，但仍建议上车实测。
3. **姿态误差建模仍是近似**：目前用"共模比例 + 独立"拆分来近似缓慢变化的姿态偏差，
   更彻底的做法是把姿态偏差建成状态量（会增加 3 维状态），本次未做。
4. **部分遮挡（单灯条）**：UV 模型天然支持单灯条观测，但当前 YOLO 只输出"完整装甲板"的 4 个角点，
   要吃到这部分收益需要检测端支持单灯条输出（传统视觉路径本来就有 `Lightbar.top/bottom`）。
5. **前哨/平衡/基地**：模型与代码路径已就绪，但没有对应录像，**未做离线验证**，需实车确认。
   合成轨迹上的回归结果见 §9：修复高度阶梯重复实现后，UV 前哨站已不劣于传统 ypd 路径，
   但**前哨站高度锚点本身的估计精度仍是两类路径共有的短板**（见 §9 遗留项）。
6. **⚠️ 无位姿 txt 的录像不能用于定量结论（重要）**
   `快/aim2、blue2、red1` 三段没有配套 `.txt`，只能进入"静止相机模式"：云台姿态取单位值、
   **时间戳按容器头标称帧率合成**。实测这个标称帧率是错的：
   - 1440 那 4 段**有真实时间戳**的录像，中位间隔是 **20.0ms（≈50fps）**，而容器标称 90fps，
     且抖动 σ=5.9ms、有 28~48 次 >1.5 倍中位的丢帧 —— 这套录制链路的容器帧率不可信；
   - 在 aim2 上扫假设帧率：120fps→预测误差中位数 26.64px，40fps→**13.77px**；
     red1：100fps→8.35px，50fps→**3.90px**；
   - 机理：dt 偏小 2~3 倍时 EKF 的匀速/匀角速度预测严重不足，**目标转速越高误差越大**
     （aim2 的 |vyaw| p90 达 14 rad/s，一帧要转 5~7°），而平移目标受影响小得多。
   用合理帧率（40fps）重跑后：

   | 录像 | base | t2m | uv | 中断(base→t2m/uv) |
   |---|---|---|---|---|
   | aim2 | 13.77 | 10.01 | **9.69** | 7 → 2 / 2 |
   | blue2 | 6.45 | **5.39** | 5.44 | 28 → 11 / 10 |
   | red1 | 3.22 | **2.31** | 2.48 | 18 → 6 / 8 |

   即：那三段之前显示的"预测误差大、中断多"**主要是时间戳误差造成的假象**；
   排序结论（uv ≈ t2m > base）不变。
   相机本身经背景平移对齐验证是**完全静止**的（最佳位移 0,0px），所以缺的是**时间戳**，不是姿态。

## 7. 怎么看 UV 观测的效果（可视化调试）

`tests/auto_aim_test.cpp` 已经内置 UV 可视化，**按 `u` 可以在运行时切换 UV / 传统观测**，同一段视频直接对比：

```bash
cd build
# 默认按 configs/demo.yaml（传统 YPD）；运行中按 u 切到 UV
./auto_aim_test ../快/2026-04-04_19-59-07

# 直接用 UV 配置启动（configs/rb_auto_aim.yaml 里有 uv_* 参数说明）
./auto_aim_test ../快/2026-04-04_19-59-07 --config-path=/tmp/F1440_uv.yaml

# 没有显示器时：每 N 帧保存一张标注图，事后看 PNG
./auto_aim_test ../快/2026-04-04_19-59-07 --config-path=/tmp/F1440_uv.yaml     --save-every=60 --save-dir=/tmp/uv_frames
```

> 注意：本机 OpenCV 4.5.4 的 `CommandLineParser` 不认 `-c value` / `-e value` 这类短选项写法，请用 `--config-path=...` / `--save-every=...` 的等号形式。

画面上能看到：

| 元素 | 含义 |
|---|---|
| 左上 `obs: UV(pixel) / YPD(world)`，`fused:N`，`nis:x.x` | 当前观测模式、本帧融合了几块装甲板、NIS |
| 灰色/绿色装甲板轮廓 + `id0..id3` | 滤波器对整车 4 块装甲板几何的**预测**；绿色是当前 `last_id` |
| **青色实心点** | 检测到的灯条端点（即 UV 观测的原始输入） |
| **红色十字** | 模型预测的灯条端点 |
| **橙色连线** | 观测↔预测残差，越短说明 UV 拟合越好 |
| `match idN corner_err x.xpx` | 关联到的装甲板编号与四角平均像素误差 |
| `L: / R:` 两行 | 两条灯条各自的 UVL 残差：角度(deg)、中心(dx,dy px)、长度(px) |
| 右上角 `zoom xN` 放大窗 | 装甲板附近放大图，用来肉眼检查端点级别的拟合 |
| 底部 `[u] switch obs mode` | 按键提示与当前模式 |

调试建议：先看 `corner_err` 和 UVL 残差的量级（正常应在 1~5px / 1~2deg 以内），
再对比 UV 与 YPD 模式下 `fused`、`nis`、以及瞄准指令的抖动；
绿色轮廓与红框贴合、橙色连线很短，就说明 UV 观测模型和相机标定是一致的。

## 8. 复现

```bash
cd build
# 单元测试
./uv_jacobian_test && ./uv_filter_test && ./uv_regression_test && ctest
# 离线 A/B（示例）
./tracker_offline_bench ../快/2026-04-04_19-59-07 /tmp/base.csv --config-path=../configs/demo.yaml
./tracker_offline_bench ../快/2026-04-04_19-59-07 /tmp/uv.csv   --config-path=/tmp/F1440_uv.yaml
# 旋转录像（无位姿 txt，自动进入静止相机模式，需 1280 内参配置）
./tracker_offline_bench ../快/red1 /tmp/red1_uv.csv --config-path=/tmp/F1280_red_uv.yaml
```

## 9. 修订记录：观测链 6 处缺陷（提交前 code review）

以下 6 项均已修复，并由 `tests/uv_regression_test.cpp` 固化（A1/A2/B）。

| 编号 | 问题 | 影响 | 修复 |
|---|---|---|---|
| A1 | `UvModel::tower_height_multiplier` 与 `RVfromFYT::tower_height_multiplier` 是两份实现，前者对无效锚点返回 0、后者按占位值 0.0 算出"低 2 级" | 前哨站刚开始跟踪（1/2 号锚点尚未学到）时，观测几何与瞄准几何差 0.2m，瞄准点偏低 0.42m | 抽出 `tower_armor_height_multiplier`（`tracking/tower_armor_geometry.hpp`）两侧共用，无效锚点一律返回 0 |
| A2 | UV 分支对前哨站一帧内顺序更新多块装甲板 → `last_id` 帧内来回跳 → 高度锚点采样窗口只有 1~2 帧；互补滤波从 0 起步（a=0.1）→ 锚点 ≈ 0.1×真实高度 | 高度阶梯方向翻转（0.1m 高的板被判成低 0.1m），瞄准点偏 0.2~0.4m | 前哨站/基地在 UV 下也**每帧只更新一块板**；`bookkeep_after_update` 增加 `kTowerArmorMinAnchorSamples=5` 的最短窗口保护 |
| B | `update_count_` 在多装甲融合下一帧内自增多次 | 语义从"帧数"变成"校正次数"，`convergened()`（前哨站位置噪声锁 0.1）、planner 开火门槛 `<500`、binocular `<70` 全部被成倍加速 | 按帧时间戳去重，每帧最多 +1 |
| C | `auto_aim_test` 的 `end-index` 与新增 `save-every` 共用短选项 `e` | OpenCV 4.5.4 把两者绑到同一键：`-e=7` 会同时改掉 end-index 并开始往 `/tmp/auto_aim_frames` 写图 | `save-every` 短选项改为 `n` |
| D | UV 模式逐帧不跑 PnP，`armor.xyz_in_world` 恒为 0，但 FFT 采样仍取该字段 | 周期性 z 加速度前馈被静默关闭（配置一旦给 `set_fft` 的程序开 UV 就会踩到） | 新增 `update_fft_sample_from_filter()`，UV 分支取 `h_armor_xyz(x, last_id).z()` |
| E | 老路径的 `break` 写在 `try_solve()` 之后 | 同帧存在第二块同名装甲且未开融合时，白跑一次 PnP + 140 步 yaw 搜索（约 100~200us/帧） | 把 `break` 提到 `try_solve()` 之前 |
| F | UV 因缺少相机几何而回退 ypd 时无任何日志；`Solver::camera_geometry()` 只要内参是 3x3 就置 `valid` | "以为在跑 UV 其实在跑 ypd"，A/B 结论失真；退化内参（fx/fy<=0 或非有限）会被当成有效几何 | 回退时告警一次；`camera_geometry()` 校验 fx/fy>0、有限性、外参有限性 |

**遗留项（未修，属两类路径共有的老问题）**：前哨站高度锚点由状态预测累加、互补滤波与
`0.16/0.05` 步进阈值耦合，锚点在首圈内偏低（本例 0.923/1.048/1.072 vs 真值 1.0/1.1/1.2），
使 `d2=0.149` 落在 `0.16` 阈值之下、2 号板被量化为 +1 级而非 +2 级。
同一段合成轨迹上传统 ypd 路径同样存在（最大误差 0.250m vs UV 0.215m），因此本次只保证
**UV 不劣于 ypd**，锚点估计精度的改造（窗口预热剔除、差分就近量化到 `kTowerArmorHeightStep` 的整数倍）
留作后续独立改动，需带录像 A/B 后再上车。
