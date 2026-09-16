# 让下发给下位机的数据频率稳定

结论先行：**光把 `sleep_for` 换成 `sleep_until` 不够**（但也不是"被相机牵着走"，见下面的更正）。当前 `plan_thread` 的周期由三段可变耗时相加而成：

1. `planner.plan(...)` 的耗时（毫秒级、方差最大）；
2. `plotter.plot(data)` 的耗时（json 构造 + dump + UDP sendto，录制时还有每点 flush）；
3. 末尾 `sleep_for(10ms)` 是"工作之后再睡"，于是周期 = 解算耗时 + 绘图耗时 + 10 ms，前两项抖多少，发送间隔就抖多少。

**更正（重要）**：`target_queue.front()` 在这段代码里**不会阻塞**。队列容量 1、溢出策略 `DropOldest`（`rb_auto_standard.cpp:60`），构造函数 push 过一次 `nullopt`，而且**全程没有任何地方调用 `pop()`**——队列里恒有 1 个元素，`front()` 每次都立即返回（只是可能返回上一帧的旧数据）。所以相机链的抖动**不影响发送节拍**，只影响数据新鲜度。真正要治的就是上面 1~3 段。

要彻底稳定，有两条路：**单线程把发送钉在节拍点上**（§2.5，前提是单轮工作 < 周期），或**把发送拆成独立线程**（§3，与工作耗时完全无关）。

---

## 0. 拆解一条循环的时间轴

`src/rb_auto_standard.cpp:70-136`（自瞄分支）：

```text
target_queue.front()   ← 立即返回（队列恒非空，见上面的更正）；取到的是"最新一帧"的拷贝
gimbal.state()         ← 短暂锁
planner.plan(...)      ← 可变：2×TinyMPC(10 次 ADMM) + 102×EKF predict + 102×rbaim
gimbal.send(...)       ← 29 B → tty 缓冲，通常几十微秒（见 §0.2）
plotter.plot(data)     ← nlohmann::json 构造 + dump + UDP sendto（+ 录制时每点 flush 落盘）
std::this_thread::sleep_for(10ms)
```

### 0.1 各段耗时的相对量级

| 来源 | 量级 | 说明 |
| --- | --- | --- |
| `planner.plan` | 毫秒级、方差最大 | 见 §4 的优化点 |
| `plotter.plot` | 几十微秒 ~ 毫秒 | `json.dump()` 每次分配；开录制时 `<< ... << flush()` 是**每点同步落盘**（`plotter.cpp:122-137`） |
| `sleep_for` | 系统性偏差 | 周期 = work + 10 ms，work 抖多少，周期就抖多少 |
| `target_queue.front()` | 拷贝开销（微秒） | 只在持锁下拷贝一个 `Target`；**不阻塞**（队列恒非空） |
| 相机链到达时刻 | 只影响**数据新鲜度**，不影响节拍 | 相机 90 fps ≈ 11.1 ms > 10 ms 时，会有部分迭代重复使用同一帧 |
| `write()` 系统调用 | **可忽略** | 见下 |

### 0.2 串口 write 基本不是瓶颈（可以先排除）

* `serial_.setTimeout(Timeout::simpleTimeout(2))`（`gimbal.cpp:41`）给出 2 ms 写超时；`unix.cc:677-747` 的 write 走 `pselect` + `::write`，**只有 tty 输出缓冲满时才会等待**，上限 2 ms。
* 460800 bps、8N1 下 29 字节的线上时间是 `29×10/460800 = 0.63 ms`；tty 默认输出缓冲 4 KB，以 5~10 ms 的周期发 29 字节**不可能填满**。
* 所以 `send()` 调用本身是几十微秒，抖动主要来自它前面那几段。

> 结论：先别动串口，先动调度结构和 `plan` 的耗时。

### 0.3 `front()` 为什么会"永不阻塞"

`front()` 只在队列**为空**时才挂起（`thread_safe_queue.hpp:104-110`）。而这段代码里：

* 队列容量 1、溢出策略 `DropOldest` ⇒ 生产者每次 push 都保证队列里有 1 个元素；
* 构造函数已经 `target_queue.push(std::nullopt)`；
* **规划线程只 `front()`，从不 `pop()`** ⇒ 元素永远不会被取走。

三者叠加的结果：**除极短的过渡瞬间外，队列恒非空，`front()` 恒立即返回**。它取到的是"队里那一个元素"——也就是最新 push 进来的那帧（DropOldest 会把旧的顶掉），但如果相机比循环慢，就会**连续多轮拿到同一帧**。

推论：

* 相机链的抖动**不进入发送节拍**，只影响"这条命令基于多旧的目标状态"；
* `front()` 的唯一代价是**持锁拷贝一个 `Target`**（含 11×11 协方差、UV 相机几何里的 `std::vector`），给相机线程的 push 注入几微秒锁等待。换成 `wait_pop()`（内部 `std::move`）更便宜，但那样就真的会阻塞在空队列上了（见 §3 的注意事项）。

> 只有当有人调用 `pop()` / `wait_pop()` / `clear()` 时，才会出现"队列空 ⇒ 阻塞"的行为。当前代码不会。

### 0.4 两个连带事实

1. **同一帧可能被反复取出重算**：相机 90 fps（11.1 ms）比 10 ms 的循环慢，于是约有 1/10 的迭代拿到的是上一帧。这不算错误——`Target` 自带时间戳，`predict(now + delay + fly)` 会把它外推到当前时刻——但要知道此时"最新观测"其实是 10~20 ms 前的。
2. **容量 1 + DropOldest 是刻意的**：丢帧、不积压，保证延迟有界。

### 0.5 退出路径：当前安全，但改成 `wait_pop_for` 后就必须配套

```cpp
// src/rb_auto_standard.cpp:229-230
quit = true;
if (plan_thread.joinable()) plan_thread.join();
```

* 当前安全：因为队列恒非空，规划线程不会卡在 `front()` 里（`tools::Exiter` 只置标志、不结束进程，`tools/exiter.cpp:11`，所以主循环会正常走到 `join()`）。
* **一旦按 §3 的建议把 `front()` 换成 `wait_pop_for(...)`，队列就可能被取空**，此时若生产者已停止 push，`join()` 会永久等待 → 进程退不掉。所以那种改法必须配套关闭队列：

```cpp
quit = true;
target_queue.close();          // 唤醒可能阻塞中的 wait_pop
if (plan_thread.joinable()) plan_thread.join();
```

* 且不能再回到 `front()`：`close()` 之后在空队列上调用 `front()` 会**抛 `std::runtime_error`**（`thread_safe_queue.hpp:108`），规划线程的 lambda 没有 catch → 直接 `std::terminate`。只加 `close()` 不改循环，会把"挂住"换成"崩掉"。仓库里 `rb_auto_aim_simulator.cpp:568-569` 是关闭队列的正确先例。

---

## 1. 先测量，别猜

在 `plan_thread` 里加一个最小的抖动统计（临时调试用）：

```cpp
auto prev_send = std::chrono::steady_clock::now();
double sum = 0, max_dev = 0; int n = 0;

// 在 gimbal.send(...) 之后：
const auto now = std::chrono::steady_clock::now();
const double us = std::chrono::duration<double, std::micro>(now - prev_send).count();
prev_send = now;
sum += us; max_dev = std::max(max_dev, std::abs(us - 10000.0)); n++;
if (n % 100 == 0) tools::logger()->info("send interval mean {:.0f}us  max|dev| {:.0f}us", sum/n, max_dev);
```

同时把 `plan` 的耗时单独量出来（`t0 = now(); plan(...); t_plan = now()-t0`）。

要看的三个数：

```text
① 发送间隔的 mean / p50 / p99 / max|dev − 目标周期|
② plan 耗时的 mean / p99 / max          ← 决定周期下限
③ 间隔与 plan 耗时的相关性              ← 若强相关，说明是"work + sleep_for"在作祟
```

---

## 2. 方案 A：绝对时刻调度（最小改动，先落地）

**原理**：把节拍点固定在 `t0 + k·period`，周期变成 `max(work, period)` 而不是 `work + period`。

已提供工具：`tools/periodic_timer.hpp`（header-only，无需改 CMake）。

```cpp
#include "tools/periodic_timer.hpp"

tools::tighten_timer_slack();                       // 收紧 timer slack（默认 50us → 1ns）
tools::PeriodicTimer tick(std::chrono::milliseconds(10));   // 100 Hz

while (!quit) {
  tick.wait_next();        // 睡到下一个绝对节拍点；超时不补发，直接对齐到未来整节拍
  // ... work ...
}
```

**实测对比**（这台开发机，2 ms 周期，1000 轮，工作量在 50~450 µs 间抖动，无 RT 优先级）：

| 调度方式 | mean | p50 | p99 | max |
| --- | --- | --- | --- | --- |
| `work + sleep_for` | 2023.6 µs | 2014.9 | 2197.9 | 2708.3 |
| `sleep_until(deadline)` | **2000.0 µs** | 1999.9 | 2162.8 | 2581.9 |

（数字是测量点取"唤醒时刻"的 wake-to-wake 间隔；单位为 µs。）

要点：

* **均值被拉回精确周期**，不再等于"周期 + 平均工作量"；这是最直观的收益。
* 残余抖动（p99 +163 µs）来自内核定时器与 CFS 调度，可用下面的手段继续压。
* **不补发**：`PeriodicTimer::wait_next()` 在迟到时跳到未来第一个整节拍点，并把跳过数记进 `missed()`。控制指令流里"少发一拍"远好于"连发两拍"（后者会让下位机看到虚假的加速度）。
* `missed() > 0` 就是"算力不够"的在线告警，建议记进 plotter。

进一步压抖动（可选，按收益排序）：

```text
1) 实时优先级：SCHED_FIFO，优先级 20~50（需要 rtprio 权限）
   systemd 服务里写：CPUSchedulingPolicy=fifo / CPUSchedulingPriority=30
2) CPU 亲和：把发送线程与相机线程分到不同核（taskset / sched_setaffinity）
3) mlockall(MCL_CURRENT|MCL_FUTURE)：避免缺页造成的毫秒级停顿
4) 关闭该核的节能调频：cpupower frequency-set -g performance
```

**方案 A 的前提：`work < period`。** 因为队列恒非空（§0.3），这里的 `work` 就是"取样 + `plan` + 绘图"这几段，不再包含等相机的时间。所以只要 `work < period`，方案 A 就能把发送间隔钉死在 period 上。

### 2.5 单线程把"发送"钉在节拍点上（哨兵 `sb_send` 循环的最小改法）

关键不是"加一个 timer"，而是**调整语句顺序，让发送紧跟在唤醒之后**：

| 顺序 | 发送间隔 | 采样→发出的数据年龄 |
| --- | --- | --- |
| 现状 `work → send → plot → sleep_for(5ms)` | `work + plot + 5ms`（全抖） | `work`（1~5 ms，最新鲜） |
| **`work → wait_next() → send → plot`** | **精确 period**（work < period 时） | ≈ period（多等了一个节拍） |
| 独立发送线程（§3） | 精确 period（与 work 无关） | ≤ period，平均 ≈ period/2 |

多出来的那点数据年龄可以忽略：6 m 处横向 0.8 m/s 的目标，10 ms 对应 8 mm ≈ 0.08°，而开火窗是 0.57°；而且 `Planner::plan` 内部用的是 `std::chrono::steady_clock::now()`（`planner.hpp:92`），延迟本来就被"预测到 now + delay + fly"吸收掉了。

**实测三种顺序的发送间隔**（本机，10 ms 周期，400 轮，工作耗时 0.5~1.8 ms 抖动）：

| 循环顺序 | mean | p50 | p99 | max\|dev\| |
| --- | --- | --- | --- | --- |
| `work → send → sleep_for(10ms)`（现状） | 11074.0 µs | 11070.1 | 11967.9 | 2394.6 µs |
| **`work → wait_next() → send`（推荐）** | **9999.9 µs** | 9997.3 | 10378.1 | 713.5 µs |
| `wait_next() → work → send`（错位） | 10001.6 µs | 10031.3 | 10577.7 | 1434.5 µs |

* 第一行：均值 = 10 ms + 平均工作量，抖动最大。
* 第二行：均值精确等于周期，残余 713 µs 是本机（有负载、无 RT 优先级）的调度抖动。
* 第三行：均值也是 10 ms，但**发送时刻 = 唤醒 + 工作量**，抖动翻倍到 1434 µs —— 证明"`wait_next()` 必须紧贴 `sb_send()`"。

改造后的循环（对应你贴的那段哨兵代码）：

```cpp
#include "tools/periodic_timer.hpp"

tools::tighten_timer_slack();                                  // 收紧 timer slack
tools::PeriodicTimer tick(std::chrono::milliseconds(10));      // 目标：严格 100 Hz

while (!quit) {
  // ① 本节拍的工作：取样 + 解算 + 组装（耗时可变，全部放在节拍点之前）
  auto target = target_queue.front();
  auto gs = gimbal.state();
  auto plan = planner.plan(target, gs.bullet_speed, gs.yaw,
                           auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
  uint8_t name = 0; float tx = 0.0f, ty = 0.0f;
  if (target.has_value()) {
    name = static_cast<uint8_t>(target->name) + 1;
    tx = target->ekf_x()[0];
    ty = target->ekf_x()[2];
  }

  // ② 睡到绝对节拍点（超时自动跳到未来整节拍，绝不补发）
  const auto lateness = tick.wait_next();      // 可记进 plotter 做在线抖动监控

  // ③ 节拍点一到立刻发：从唤醒到写出只隔一次函数调用
  gimbal.sb_send(plan.control, plan.fire,
                 plan.yaw, plan.yaw_vel, plan.yaw_acc,
                 plan.pitch, plan.pitch_vel, plan.pitch_acc,
                 tx, ty, name);

  // ④ 绘图放在发送之后：它的抖动不再影响发送时刻
  //    （但仍占本节拍预算，要求 work + plot + send < 10 ms；录制的每点 flush 建议移出该线程）
  nlohmann::json data;
  // ... 填 data ...
  plotter.plot(data);
}
```

要点：

* **`wait_next()` 必须紧接在 `sb_send()` 之前**。若把它放在循环开头（`wait → work → send`），唤醒时刻虽然精确，但发送时刻 = 唤醒 + work，抖动又回来了。
* `plotter.plot` 必须放在发送**之后**；它的 `json.dump()` + `sendto`（录制时还有每点 `flush` 落盘）是本节拍里方差最大的一段。如果 `work + plot + send` 接近 10 ms，就把它挪到独立低优先级线程（`ThreadSafeQueue<json>` 消费）。
* `tick.missed()` 若持续增长，说明单轮工作已经超过 10 ms —— 此时**单线程方案无法再保证 10 ms**，只能上 §3 的发送线程（或按 §4 削 `plan` 耗时）。
* 注意这会把频率从原来的 `1/(5ms + work)` ≈ 110~140 Hz 改成固定 100 Hz。若你想要的是"更快而且稳"，把周期设成 5 ms 并确保 `work < 5 ms`。
* `sb_send` 走的是 `write_frame`（**不计算 CRC**，`gimbal.cpp:114-118`），帧长 36 字节（1+1+24+8+1+1）：460800 bps 下线上时间 0.78 ms，100 Hz 占 7.8% 带宽，无需担心。

### 2.6 `wait_next()` 是怎么做到的

它只做了三件事：**记住绝对节拍点、用绝对时刻睡、超时就跳拍**。

```cpp
const auto now = Clock::now();
if (now >= next_) {                       // ② 上一轮超时了
  const auto behind  = now - next_;
  const auto skipped = behind / period_ + 1;   // 至少要跳 1 个整节拍
  missed_ += skipped;
  next_ += period_ * skipped;             // 对齐到"未来第一个整节拍"，绝不连发补拍
}
std::this_thread::sleep_until(next_);     // ① 睡到绝对时刻
const auto lateness = Clock::now() - next_;    // ③ 迟到量，供监控
next_ += period_;                         // 节拍点永远走等差数列 t0 + k·P
return lateness;
```

**① 为什么"绝对时刻"能吃掉工作耗时。** `std::this_thread::sleep_until` 的实现在 libstdc++ 里就是这样（`/usr/include/c++/12/bits/this_thread_sleep.h:88-107`）：

```cpp
template<typename _Clock, typename _Duration>
inline void sleep_until(const chrono::time_point<_Clock, _Duration>& __atime)
{
  auto __now = _Clock::now();
  if (_Clock::is_steady) {                 // steady_clock 走这一支
    if (__now < __atime) sleep_for(__atime - __now);   // ← 剩余量 = 目标 − 现在
    return;
  }
  while (__now < __atime) { sleep_for(__atime - __now); __now = _Clock::now(); }
}
```

而 `sleep_for` 最终落到 `nanosleep(&ts, &ts)`（hrtimer，纳秒级，不受 1/HZ 限制）：

```cpp
while (::nanosleep(&__ts, &__ts) == -1 && errno == EINTR) { }   // 被信号打断就用剩余时间重试
```

关键差别在一行：

```text
sleep_for(P)      ：内核从"调用那一刻"开始数 P      → 总周期 = 工作耗时 + P
sleep_until(next) ：剩余 = next − now（每次重新算）  → 工作耗时被自动扣掉
```

`std::this_thread::sleep_until` 每次都把"已经花掉的时间"从睡眠量里减掉，所以只要 `next_` 是等差数列，唤醒时刻就与工作耗时无关。

**常见疑问："之前没取过时间戳，它怎么知道前面耗时多久？"——它不需要知道。** `wait_next()` 只需要两样东西：**现在几点**、**我记的那个节拍点是几点**，两者的差就已经包含了"上一拍之后花掉的全部时间"（工作、绘图、被抢占……全都算在里面）。所以：

* `next_` 是唯一需要保存的状态（一个 `time_point`），构造时初始化为 `now + period`；
* 每次 `wait_next()` 的第一行 `const auto now = Clock::now();` 就是那个"时间戳"——它每轮都有，而且 `steady_clock::now()` 走 vDSO，本机实测 **11.4 ns/次**，比一次函数调用还便宜；
* 真正做减法的是 `sleep_until` 内部那一句 `sleep_for(__atime - __now)`；
* 正常时 `behind = now − next_ < 0`（还早，差值就是还要睡多久）；超时 `behind > 0`（已经晚了，差值就是要跳几拍）。

类比：设了 7:00 的闹钟，醒来时你不需要知道"自己睡了多久"，只需要看现在几点 —— 而"现在几点"每轮都是现读的。

一次真实运行的内部量（10 ms 周期，第 4 轮故意让工作耗时 14 ms；`next−t0` 是**跳拍后**的值）：

```text
 k  work(仅供对照)  now-t0   next-t0   behind=now-next   本轮间隔   醒来迟到
 1     2.0 ms        2.01     10.00      -7.99 ms      10.16 ms    +155.9 us
 2     3.0 ms       13.20     20.00      -6.80 ms      10.01 ms    +168.4 us
 3     2.0 ms       22.19     30.00      -7.81 ms      10.02 ms    +186.7 us
 4    14.0 ms       44.21     50.00      +4.21 ms      20.16 ms    +350.8 us   ← 超时，跳一拍
 5     2.0 ms       52.37     60.00      -7.63 ms       9.78 ms    +134.2 us
 6     2.0 ms       62.15     70.00      -7.85 ms      10.01 ms    +139.8 us
 7     3.0 ms       73.16     80.00      -6.84 ms       9.99 ms    +132.8 us
 8     2.0 ms       82.15     90.00      -7.85 ms      10.11 ms    +245.7 us
```

`work` 那一列只出现在这张表里供对照，**从不参与任何运算**；决定一切的自始至终只有 `(now, next_)`。
需要区分"调度"和"监控"：想统计工作量（用于优化 `plan`）确实要自己加 `t_start/t_end`，但那是**监控**，不是调度——调度只需要记住截止时刻。

**② 一个常被忽略的性质：迟到的"均值"不影响间隔。** `nanosleep` 只能晚醒、不能早醒，所以 `lateness ≥ 0`。设 `wake_k = next_k + late_k`、`next_k = t0 + k·P`，则

```text
interval_k = wake_k − wake_(k−1) = P + (late_k − late_(k−1))
```

相邻两拍的迟到量相减会**前后抵消**：即使平均迟到 65 µs，平均间隔仍然是精确的 P（这就是为什么实测 mean = 9999.9 µs，而同期测出的平均迟到是 +23 µs）。**常数迟到只是把所有发送时刻一起平移，不改变频率。**本机实测迟到分布：

```text
min 1.45 µs   p50 25.00 µs   p99 377.03 µs   max 851.86 µs   mean 64.53 µs   负值 0/3000
```

（本机有负载、无 RT 优先级；残余抖动就是这些迟到量的**差分**。）

**③ 超时策略。** `behind / period_ + 1` 是"整周期个数 + 1"，保证新节拍点严格落在未来：

```text
next_new = next_ + (⌊behind/P⌋ + 1)·P  ≥  next_ + behind + P = now + P > now
```

于是模型很确定：**要么按 P 发，要么跳过整数个 P（间隔变成 2P、3P…），永不出现"连发两拍"**，跳过的数量记在 `missed()` 里当算力告警。

**④ 还能更准的三个旋钮**（按收益排序）：

| 旋钮 | 作用 |
| --- | --- |
| `tighten_timer_slack()`（`PR_SET_TIMERSLACK`） | 内核默认会把定时器合并、最多推迟 50 µs 才唤醒。本机实测 `sleep_for(1us)`：默认 slack **51.58 µs/次**，设为 1 ns 后 **2.07 µs/次**（相差 25 倍） |
| `SCHED_FIFO` + CPU 亲和 + `isolcpus` | 定时器到期后线程还要被调度上 CPU；CFS 下这能占掉几百 µs（实测 max 852 µs），实时优先级能压到几十 µs |
| `mlockall`、`performance` 调频、关 C-state | 消除缺页与降频带来的毫秒级停顿 |

**⑤ 两个边界。** `steady_clock` 在 Linux/libstdc++ 上就是 `clock_gettime(CLOCK_MONOTONIC)`，不受 NTP 校时影响（换成 `system_clock` 会被时间跳变带偏，而且 libstdc++ 会走上面那个 `while` 分支）。另外注意 `is_steady` 那一支**只睡一次、不做二次校验**：万一被信号提前唤醒，`sleep_for` 内部已经用"剩余时间重试"兜住了，所以实际到达时刻总是 ≥ 节拍点。

### 2.7 为什么不直接在循环里写 `std::this_thread::sleep_until(next_)`

因为 `wait_next()` 包的不是"睡眠精度"，而是**节拍点的记账**。`sleep_until` 只负责"睡到某个绝对时刻"，剩下的三件事它不管，而这三件事恰好都是会出事的：

**(1) 超时后裸 `sleep_until` 会"连发"。** 手写成

```cpp
auto next = std::chrono::steady_clock::now() + P;
while (...) {
  work();                        // 万一这一轮超过 P
  next += P;
  std::this_thread::sleep_until(next);   // ← next 已经在过去
  send();
}
```

当 `work > P` 时 `next` 落在过去，而 libstdc++ 的 `sleep_until` 在 `__now < __atime` 为假时**直接 return**（`this_thread_sleep.h:94-99`）——于是循环空转、背靠背连发，直到 `next` 追上 `now`。实测（10 ms 周期，每 20 轮插一次 25 ms 的慢工作）：

| 写法 | 最小间隔 | p50 | 最大间隔 | 间隔 < 5 ms 的"连发"次数 |
| --- | --- | --- | --- | --- |
| `next += P; sleep_until(next)` | **1.42 ms** | 9.99 ms | 29.69 ms | **18 次** |
| `PeriodicTimer::wait_next()` | 9.66 ms | 10.00 ms | 39.83 ms | **0 次** |
| 迟到就 `next = now + P` | 9.48 ms | 10.01 ms | 39.67 ms | 0 次 |

连发对下位机比跳拍危险得多：如果它用相邻帧的到达时间算 dt，会看到虚假的加速度尖峰；如果它对角度做差分估速度，会算出 0 或反向。

**(2) 超时策略必须显式选一个。** 至少有两种合理策略，裸 `sleep_until` 的默认行为（立即返回）几乎肯定不是你想要的那个：

| 策略 | 代码 | 行为 | 长期频率 |
| --- | --- | --- | --- |
| A 回到全局网格（本工具采用） | `next += ⌈(now−next)/P⌉·P` | 本次间隔变成整数倍 P（实测 30/40 ms），之后仍落在原来的网格上 | 严格 1/P |
| B 重置相位 | `next = now + P` | 本次间隔 = 超时耗时 + P，但相位丢失 | 略低于 1/P |

**(3) 忘了推进 `next_` 就是死循环。** 漏掉 `next += period` 会让 `sleep_until` 永远立即返回、线程 100% 占核空转。把它封进对象后，`next_` 的推进、不变量（永远在未来、永远在网格上）都在一处，多个循环也能共享同一套语义。

另外两个附带的收益：

* **可观测性**：`lateness` 是**领先指标**（还没开始跳拍时就能看到余量变薄），`missed()` 是**滞后指标**（已经跳了）。建议每 N 帧记一次 p99，而不是每帧 plot。
* **返回值可以忽略**：不关心抖动监控时直接写 `tick.wait_next();` 即可；`const auto lateness = ...` 只是为了把它喂给统计。

**等价性提醒**：正常路径上 `wait_next()` 与手写 `sleep_until(next_)` 落到同一条 `nanosleep`，精度完全一样——**精度来自 `sleep_until`，封装买的是边界行为和语义一致性**。如果你的循环只有一个、工作耗时恒定且确定小于周期，手写也完全可以；一旦耗时可变或有多个循环，封装更省事。

**最后**：若 `missed()` 持续增长（不是偶尔跳一拍），说明单轮工作已经常态超过周期 —— 这时正确的做法不是把周期调大，而是把发送拆成独立线程（§3），那样连"跳拍"都不会发生。

---

## 3. 方案 B：解算与发送解耦（推荐，根治）

把"谁算"和"谁发"分开：

```text
[主线程 相机]  ──push──▶  target_queue(1)
                              │
                    ┌─────────▼──────────┐
                    │  plan_thread       │  节拍随意（例如 100 Hz，用 PeriodicTimer）
                    │  plan() → Plan      │  算完就把"待发帧"发布到共享槽
                    └─────────┬──────────┘
                              │ publish(Plan)  ← 只做一次 8 float + 2 bool 的拷贝
                    ┌─────────▼──────────┐
                    │  send_thread       │  PeriodicTimer(5ms) = 200 Hz，绝对节拍
                    │  取快照 → send()    │  只做 memcpy + write(29B)，绝不阻塞
                    └─────────┬──────────┘
                              ▼
                          下位机
```

**这样发送周期只取决于时钟，与 plan 耗时、相机抖动、绘图耗时全部无关。** 最坏情况只是"数据旧了一个解算周期"（100 Hz → 10 ms），而这对绝对角 + 前馈的指令语义完全可接受。

可直接贴的骨架：

```cpp
#include <mutex>
#include "tools/periodic_timer.hpp"

struct PlanSlot {                 // 发布槽：只存下位机真正需要的量
  std::mutex m;
  bool control = false, fire = false;
  float yaw = 0, yaw_vel = 0, yaw_acc = 0;
  float pitch = 0, pitch_vel = 0, pitch_acc = 0;
  bool fresh = false;
};
PlanSlot slot;

// —— 解算线程：算完就发布，节拍可以随便抖 ——
auto plan_thread = std::thread([&]() {
  tools::PeriodicTimer tick(std::chrono::milliseconds(10));
  while (!quit) {
    tick.wait_next();
    if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF)
      continue;                                   // 打符模式由主循环发送，见下
    auto target = target_queue.wait_pop();        // 用 wait_pop 取走，避免 front() 里的整对象拷贝
    auto gs = gimbal.state();
    auto plan = planner.plan(target, gs.bullet_speed, gs.yaw,
                             auto_aim::Planner::ShootStrategy::rbSuppressiveFire);
    {
      std::lock_guard<std::mutex> lk(slot.m);
      slot.control = plan.control; slot.fire = plan.fire;
      slot.yaw = plan.yaw; slot.yaw_vel = plan.yaw_vel; slot.yaw_acc = plan.yaw_acc;
      slot.pitch = plan.pitch; slot.pitch_vel = plan.pitch_vel; slot.pitch_acc = plan.pitch_acc;
      slot.fresh = true;
    }
    // 绘图/记录留在这里（它再慢也不影响发送节拍）
  }
});

// —— 发送线程：全程只读快照 + 写串口，任何情况下都不阻塞 ——
auto send_thread = std::thread([&]() {
  tools::tighten_timer_slack();
  tools::PeriodicTimer tick(std::chrono::milliseconds(5));   // 200 Hz
  float yaw = 0, yv = 0, ya = 0, pit = 0, pv = 0, pa = 0;
  bool ctl = false, fire = false;

  while (!quit) {
    const auto lateness = tick.wait_next();
    if (mode.load() == io::GimbalMode::SMALL_BUFF || mode.load() == io::GimbalMode::BIG_BUFF) continue;

    {
      std::unique_lock<std::mutex> lk(slot.m, std::try_to_lock);
      if (lk.owns_lock() && slot.fresh) {       // try_lock 失败就重发上一帧
        ctl = slot.control; fire = slot.fire;
        yaw = slot.yaw; yv = slot.yaw_vel; ya = slot.yaw_acc;
        pit = slot.pitch; pv = slot.pitch_vel; pa = slot.pitch_acc;
        slot.fresh = false;
      }
    }
    gimbal.send(ctl, fire, yaw, yv, ya, pit, pv, pa);
    // 可选：把 lateness / tick.missed() 记进 plotter（低频，别每帧写盘）
  }
});
```

几个必须注意的点：

1. **发送线程里不要有任何可能阻塞的东西**：不要 `logger()`（会锁 + 写终端）、不要 `plotter.plot()`（`json.dump` + `sendto` + 可能每点 flush）、不要 `target_queue.front()`。
2. **`try_lock` 失败就重发上一帧**：发送线程永不等待解算线程。因为发布端持锁时间只有一次 8 float 拷贝，正常永远抢得到。
3. **`fresh` 标志**：不想在解算停摆时一直重发同一帧，可以看 `fresh`；但对"绝对角保持"的语义来说重发是安全的。
4. **打符模式**：现在打符由主循环直接 `gimbal.send`（`rb_auto_standard.cpp:188-190`）。两条发送路径并存时，`Gimbal::send` 内部各自组装局部帧 + `serial_.write` 自带写锁，所以不会串帧（`gimbal.hpp:167-170` 已经说明）；但更彻底的做法是把打符命令也走同一个 `PlanSlot`，由发送线程统一发出。
5. **发送频率 vs 解算频率**：200 Hz 发送 + 100 Hz 解算 = 每帧发两次。对"绝对角 + 前馈"的下位机是安全的（值不变）；**但如果下位机对收到的角度做差分来估速度，重复帧会被算成 0 速度**——此时要么发送频率 = 解算频率，要么在帧里加序号/时间戳（见 §5）。

---

## 4. 方案 C：削平 `plan` 的耗时与方差（顺手做，收益是"数据更新鲜"）

即使上了方案 B，缩短 plan 耗时仍能让命令更新鲜、并降低 `missed()`：

| # | 位置 | 问题 | 改法 |
| --- | --- | --- | --- |
| 1 | `planner.hpp:75,114,116,120` | `plan(Target target, ...)`、`rbplan(Target target, ...)` **按值传参**，每 10 ms 拷贝一次含 11×11 协方差与 UV 相机几何的 `Target`（`rbplan` 里还有 `Target target_at_bullet = target;`） | 改 `const Target &`，只在需要推进时间时做一次局部拷贝 |
| 2 | `target.cpp:411` 与 `:431` | `get_recent_armor_xyzad()` 里 `armor_xyza_list()` **调了两次**（局部变量一次 + range-for 一次），每次分配一个 `std::vector<Vector4d>`；而 `rbget_trajectory` 每个 plan 调它 **102 次** ⇒ 200+ 次 vector 分配 | 复用同一个局部 vector |
| 3 | `rv_from_fyt.cpp:63-105` | `predict_model` **每次调用都新建** `Eigen::MatrixXd F(11,11)`、`Q(11,11)`、`B(11,3)`（动态尺寸 = 堆分配）；每个 plan 调 102 次 | 换成固定尺寸 `Eigen::Matrix<double,11,11>`（栈上），或把 `F`/`B` 的常数块与 `Q` 的模板预计算好 |
| 4 | `rb_auto_standard.cpp:129` | `plotter.plot(data)` 在控制路径上：nlohmann::json 构造 + `dump()` + `sendto`，开录制时**每点 `flush()` 落盘** | 留在解算线程（方案 B 后无害）；若要稳定解算节拍，则丢进 `ThreadSafeQueue<json>` 由低优先级线程消费 |
| 5 | `planner_mpc.cpp:35,65` | `max_iter = 10` 次 ADMM；而加速度约束几乎不活跃（实测需求 ~0.5 rad/s²，限幅 50/100） | 降到 3~5 次，先离线对比 `plan_yaw` 的差异 |
| 6 | `planner.hpp:17-19` | 时域 100 拍（1 s），但只输出第 50 拍 | 若确认远端参考只用于保证速度/加速度连续，可缩到 30~40 拍并改输出索引，求解量降 60% |

先做 1~3（纯机械改动、无行为风险），4~6 需要离线 A/B。

---

## 5. 波特率与帧率预算

帧长 29 字节、8N1（10 bit/字节）⇒ 290 bit/帧：

| 发送频率 | 占用 460800 bps | 评价 |
| --- | --- | --- |
| 100 Hz | 6.3% | 富余 |
| 200 Hz | 12.6% | 推荐 |
| 500 Hz | 31% | 可以 |
| 1000 Hz | 63% | 不建议（RX 方向虽独立，但没有余量应对重传/抖动） |
| 理论上限 | ~1589 Hz | `460800/290` |

**要加时间戳/序号的话注意**：`VisionToGimbal` 目前 29 字节，CRC 只覆盖**前 26 字节**（`gimbal.cpp:139-152`），且下位机的 `check_crc16(buf, 29)` 是按固定长度算的。任何帧长变化都必须两侧同步改，否则表现为"所有帧被静默丢弃"。`static_assert(sizeof(VisionToGimbal) <= 64)` 说明协议还有扩展余量。

---

## 6. 先确认下位机的契约（决定"稳定"要多稳）

| 下位机怎么用这三个量 | 对发送的要求 |
| --- | --- |
| 当**绝对角参考 + 前馈**，自己跑 1 kHz 电流/速度环 | 稳定不是硬要求，**新鲜度与延迟**更重要；200 Hz 稳定发送已足够，抖动只影响它的前馈质量 |
| 对收到的**角度做差分**估速度 | **必须严格等间隔**，且不能发重复帧（会算出 0 速度）⇒ 发送频率 = 解算频率，或在帧里带序号/时间戳让它自己算 dt |
| 有**看门狗超时**（如 20 ms 无帧则停转） | 稳定发送是安全要求 ⇒ 必须方案 B，且发送线程优先级最高 |

建议先去问电控这一条，再决定 200 Hz 还是"发送频率 = 解算频率"。

---

## 7. 验收指标

```text
硬指标（方案 A/B 后应达到）：
  发送间隔 mean  = 目标周期 ±1%
  发送间隔 p99   ≤ 目标周期 × 1.1
  发送间隔 max   ≤ 目标周期 × 1.5      （无 RT 优先级；有 RT 时应 ≤ ×1.05）
  PeriodicTimer::missed() 长期为 0

软指标（数据质量）：
  命令帧 age 分布（发送时刻 − plan 完成时刻）：p99 ≤ 1 个解算周期
  plan 耗时 p99（用于判断是否需要 §4 的优化）

回归判据：
  离线回放（tests/planner_offline_replay.cpp）上 plan_yaw 序列与改动前一致
  （§4 的 5/6 项会改变数值，必须单独 A/B）
```

---

## 8. 落地记录（本次改造）

所有"解算 + 向下位机下发"的线程循环都已改成 §2.5 的形式：**算完 → `tick.wait_next()` → 立刻下发 → 再绘图**。

| 文件 | 周期 | 下发调用 |
| --- | --- | --- |
| `src/rb_auto_standard.cpp` | 10 ms | `gimbal.send` |
| `src/rb_auto_standard_async.cpp` | 10 ms | `gimbal.send` |
| `src/rb_auto_standard_debug.cpp` | 10 ms | `gimbal.send` |
| `src/rb_auto_standard_debug_async.cpp` | 10 ms | `gimbal.send` |
| `src/hero.cpp` | 10 ms | `gimbal.send` |
| `src/rb_rv_aim_debug.cpp` | 10 ms | `gimbal.send` |
| `src/rbnx_auto_aim_debug.cpp` | 10 ms | `gimbal.send` |
| `src/rbnx_auto_aim_debug_async.cpp` | 10 ms | `gimbal.send` |
| `src/rb_auto_aim_debug.cpp` | 5 ms | `gimbal.sb_send` |
| `src/rb_auto_aim_debug_fft.cpp` | 5 ms | `gimbal.send` |
| `src/rb_auto_aim_debug_async.cpp` | 1 ms | `gimbal.send` |
| `src/omni_perception.cpp` | 1 ms | `gimbal.omni_send` |
| `src/sb_omn_aim_mpc.cpp` | 10 ms | `gimbal.sb_send` |
| `src/sentry_omn_aim_mpc.cpp` | 10 ms | `gimbal.sb_send` |
| `src/sentry_binocular_aim_mpc.cpp` | 10 ms | `gimbal.sb_send` |
| `src/rb_binocular_aim_debug.cpp` | 10 ms | `gimbal.send` |
| `src/rb_binocular_aim_debug_async.cpp` | 10 ms | `gimbal.send` |
| `tasks/auto_aim/aiming/commandgener.cpp` | 2 ms | `cboard_.send` |

统一改动内容：

1. 新增 `#include "tools/periodic_timer.hpp"`；
2. 线程入口处 `tools::tighten_timer_slack();` + `tools::PeriodicTimer tick(<原 sleep 时长>);`；
3. `const auto lateness = tick.wait_next();` 紧贴在下发调用之前；
4. 删掉循环末的 `sleep_for`；打符/无目标等 else 分支改成 `tick.wait_next();`（保持节拍，不再用相对睡眠）；
5. 有 plotter 的循环把 `send_late_us`（本次迟到量，µs）与 `send_missed`（累计跳拍数）记进 json，用于在线判断"是不是快超预算了"。

### 8.1 这两个字段分别是什么

```cpp
const auto now = Clock::now();
if (now >= next_) {                       // 上一轮已经超时
  const auto behind  = now - next_;
  const auto skipped = behind / period_ + 1;
  missed_ += skipped;                     // ← send_missed 的来源（累计，单调不减）
  next_ += period_ * skipped;             // 跳到未来整节拍
}
std::this_thread::sleep_until(next_);
const auto lateness = Clock::now() - next_;   // ← send_late_us 的来源（本次）
next_ += period_;
```

| 字段 | 含义 | 单位/性质 | 正常值 | 异常表现 |
| --- | --- | --- | --- | --- |
| `send_late_us` | **本次**唤醒比节拍点晚多少（唤醒迟到量） | µs，每拍一个瞬时值 | 本机无 RT 优先级实测 min 1.5 / p50 25 / p99 377 / max 852 µs；上 RT 优先级后应 < 100 µs | 均值恒定偏大**无害**（见下），**忽大忽小**才是抖动；偶发毫秒级尖峰说明被抢占或缺页 |
| `send_missed` | 从 `PeriodicTimer` 构造起**累计**被跳过的节拍数 | 个，单调不减 | 一条水平线 | 出现台阶 ⇒ 那一拍的单轮工作超过了周期，发送间隔变成 2P/3P…；台阶密集 ⇒ 该上独立发送线程（§3） |

两个容易误读的点：

1. **`send_late_us` 的均值不影响发送间隔，它的"差分"才影响。** 因为 `interval_k = P + (late_k − late_{k−1})`，常数迟到只是把所有发送时刻一起平移（§2.6）。所以看到"平均迟到 150 µs"不用管，要看它的 p99 与最大值；真正对下位机有害的是相邻两拍迟到量之差。
2. **`send_missed` 是累计量，不是本次是否跳拍。** 想知道"这一秒跳了几拍"要看它的**斜率**（或自己在外部做差分再记一个 `send_missed_delta`）。

它们**不是**：plan 的耗时、两次发送的间隔、命令数据的年龄。想看 plan 耗时必须自己在 `planner.plan(...)` 前后各取一次时间戳单独统计（那是"监控"，见 §2.6 末尾）。

`send_late_us` 还是**领先指标**：它开始频繁变大（但 `send_missed` 还没动）时，说明单轮工作已经接近周期预算；等 `send_missed` 开始涨就已经在丢拍了。

**有意不改的 `sleep_for`**（它们不是周期性下发节拍）：

| 位置 | 用途 |
| --- | --- |
| 主循环里 `if (img.empty())` 之后的 `sleep_for(1ms)` | 相机没出图时的重试等待 |
| `sentry_binocular_aim_mpc.cpp:67` 的 `sleep_for(2s)` | 启动等待 |
| `rb_auto_aim_debug_fft.cpp` / `rb_binocular_aim_debug_async.cpp` / `rb_auto_aim_simulator.cpp` 的 `sleep_for(50ms)×5` | 退出前的等待 |
| `io/`（相机重连、CAN）、`tools/gimbal_calib.cpp` | 设备等待/标定节拍 |
| 主循环（非下发线程）里的 `sleep_for(1ms)` | 相机帧驱动，本来就没有固定周期 |

**注意两点行为变化**：

* 周期从"`sleep` + 工作耗时"（例如 10 ms + 3 ms ≈ 77 Hz）变成**严格 10 ms（100 Hz）**，下行帧率会升高，带宽占用见 §5（29 B/帧 @100 Hz = 6.3%）。
* 数据年龄（采样→发出）从 `work` 变成约一个周期（§2.5 的表），折合 6 m 处 0.8 m/s 目标 ≈ 0.08°，相对 0.57° 的开火窗可忽略。

**验证方式**（不依赖上车）：

```bash
/tmp/syn.sh src/<file>.cpp                        # 用 compile_commands.json 的原始命令做 -fsyntax-only
cmake --build build --target <target> -j4         # 真实增量编译（每个 src/*.cpp 是一个独立可执行目标）
```

两个环境注意点：

* `rbnx_auto_aim_debug*` 两个目标在 `CMakeLists.txt` 的 `if(TENSOR_RT_MAKE)` 里，而本机是 `TENSOR_RT_MAKE=OFF / OPENVINO_MAKE=ON`（未装 TensorRT），所以这两个目标在当前配置下**不参与构建**，`compile_commands.json` 里也没有条目；已用同链接集的兄弟目标（`rb_rv_aim_debug`）的编译命令行做等价语法检查。
* `sentry_omn_aim_mpc` / `sentry_binocular_aim_mpc` 的 `add_executable` 在 `CMakeLists.txt:420-427` 是**注释掉的**，当前不参与构建；这两个文件同样只做了语法检查。
* `build/compile_commands.json` 是构建产物，若目标集合变化需重新生成：`cd build && cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON .`。

---

## 附：本次给出/涉及的文件

| 文件 | 作用 |
| --- | --- |
| `tools/periodic_timer.hpp` | 新增。绝对时刻周期调度器 + `tighten_timer_slack()` |
| `src/rb_auto_standard.cpp` | 已改造的参考实现（`work → wait_next → send → plot`） |
| `tools/thread_safe_queue.hpp:104-110` | `front()` 阻塞且返回整对象拷贝（本项目恒非空，见 §0.3） |
| `tools/plotter.cpp:122-137,348-355` | 每次 plot 都 `json.dump()` + `sendto`，录制时每点 flush |
| `io/gimbal/gimbal.cpp:41,139-152` | 串口超时设置与 CRC 覆盖长度 |
| `tasks/auto_aim/aiming/planner/planner_mpc.cpp:35,65` | TinyMPC `max_iter = 10` |
