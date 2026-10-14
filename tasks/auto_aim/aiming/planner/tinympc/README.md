# TinyMPC

## 作用

这里是项目内置的 TinyMPC 求解器。它解决有限时间域线性模型预测控制问题，并支持状态、输入、锥和线性约束。

## 主要类型

- `TinySolver`：聚合缓存、设置、工作区和最终解。
- `TinyWorkspace`：保存动力学矩阵、状态/输入序列、参考轨迹、约束和 ADMM 中间变量。
- `TinyCache`：保存 rho、`Kinf`/`Pinf`/`Quu_inv` 等 Riccati 预计算，以及自适应 rho 的敏感度矩阵。
- `TinySettings`：收敛阈值、最大迭代次数和约束开关，含自适应 rho 相关字段（`adaptive_rho*`）。
- `TinySolution`：保存求解状态、迭代次数、状态轨迹 `x` 和控制轨迹 `u`。

## 主要函数

- `tiny_setup()`：根据 A/B/f、Q/R、维度和时间域创建求解器。
- `tiny_set_x0()`：设置当前初始状态。
- `tiny_set_x_ref()`、`tiny_set_u_ref()`：设置期望状态和控制轨迹。
- `tiny_set_bound_constraints()`：设置状态与输入上下界。
- `tiny_set_linear_constraints()`、`tiny_set_cone_constraints()`：配置额外约束。
- `tiny_solve()`：执行优化并把结果写入求解器工作区。
- `tiny_cleanup()`：释放 `tiny_setup()` 创建的资源。
- `tiny_update_settings()`：调整精度、迭代次数和约束开关。

## 原理

MPC 在一个有限时间域中最小化参考轨迹误差和控制代价，同时满足离散动力学及约束。TinyMPC 使用 Riccati 递推处理线性二次部分，并通过 ADMM 迭代处理边界和其他约束。

普通瞄准策略不应直接修改这里。调整状态维度或 API 后，必须同步检查上一级 `Planner` 的 A/B 矩阵、Q/R、参考轨迹尺寸和输出下标。
