# 从零开始学会部署自瞄

## 项目树
```bash
sp_vision_25rbclone
├── assets         // 包含demo素材、网络权重等
│   └── ...
├── calibration    // 标定相关程序
│   ├── calibrate_camera.cpp             // 相机内参标定程序
│   ├── calibrate_handeye.cpp            // 手眼标定程序
│   ├── calibrate_robotworld_handeye.cpp // 手眼标定程序（同时计算标定板位置）
│   └── capture.cpp                      // 相机标定数据采集程序
├── CMakeLists.txt // CMake配置文件
├── configs        // 每台机器人的YAML配置文件，见 configs/README.md
│   └── ...
├── evaluation     // 离线评估工具与原始数据（当前只在本地保留，被 .gitignore 排除）
│   └── armor_model_eval // 装甲板模型能力评估
├── io             // 硬件抽象层，见 io/README.md
│   ├── camera     // 相机统一接口、图像处理和厂商驱动
│   ├── gimbal     // 云台、C 板和控制命令通信
│   ├── serial     // 串口库
│   └── README.md
├── scripts        // setup_env.sh：一键装依赖、检查环境并编译
├── sp_example     // 早期程序的归档（1.cpp、sentry/uav/mt_* 等），不参与编译
├── sp_ws          // ROS2 消息包（sp_msgs：AutoaimTargetMsg、EnemyStatusMsg），按需 colcon build
├── src            // 应用层，调试/上场代码
|   ├── hero.cpp                        // 26赛季英雄专用
|   ├── omni_perception.cpp             // 全向感知单独运行程序
|   ├── rb_auto_aim_debug.cpp           // 26赛季步兵、哨兵通用aim debug
|   ├── rb_auto_aim_debug_async.cpp     // aim debug（异步版）
|   ├── rb_auto_aim_debug_fft.cpp       // 带FFT（前哨站等周期运动）的aim debug
|   ├── rb_auto_aim_simulator.cpp       // ROS2仿真输入的自瞄程序（尚未接入CMake，见文末说明）
|   ├── rb_auto_standard.cpp            // 步兵上场程序
|   ├── rb_auto_standard_async.cpp      // 步兵上场程序（异步版）
|   ├── rb_auto_standard_debug.cpp      // 步兵上场程序debug
|   ├── rb_auto_standard_debug_async.cpp// 步兵上场程序debug（异步版）
|   ├── rb_binocular_aim_debug.cpp      // 26赛季双目自瞄debug
|   ├── rb_binocular_aim_debug_async.cpp// 26赛季双目自瞄debug（异步版）
|   ├── rb_rv_aim_debug.cpp             // 换RV识别模型的aim debug
|   ├── rbnx_auto_aim_debug.cpp         // NX上的aim debug
|   ├── rbnx_auto_aim_debug_async.cpp   // NX上的aim debug（异步版）
|   ├── sb_omn_aim_mpc.cpp              // 26赛季步兵/哨兵通用，自瞄+全向感知
|   ├── sentry_binocular_aim_mpc.cpp    // 26赛季双目视觉版哨兵
|   └── sentry_omn_aim_mpc.cpp          // 26赛季全向感知版哨兵
├── tasks          // 功能层，见3.4软件架构
│   ├── auto_aim       // 自瞄相关算法实现，见 tasks/auto_aim/README.md
│   │   ├── model      // 公共数据结构
│   │   ├── detection  // 图像识别和推理后端
│   │   ├── geometry   // 装甲板位姿解算
│   │   ├── tracking   // 目标跟踪和滤波
│   │   └── aiming     // 瞄准、规划和火控
│   ├── auto_buff      // 打符相关算法实现，见 tasks/auto_buff/README.md
│   │   └── ...
│   └── omniperception // 全向感知相关算法实现，见 tasks/omniperception/README.md
│       └── ...
├── tests    //用于测试的代码
│   ├── auto_aim_test.cpp         // 自瞄录制视频测试程序
│   ├── auto_aim_test_async.cpp   // 自瞄录制视频测试程序（异步版）
│   ├── auto_buff_test.cpp        // 打符录制视频测试程序
│   ├── camera_detect_test.cpp    // 识别器测试程序（工业相机）
│   ├── camera_image_processing_test.cpp // 相机图像处理（Gamma/降噪）测试
│   ├── camera_test.cpp           // 相机测试程序
│   ├── cameras_test.cpp          // 多相机测试程序
│   ├── 4cmaera_test.cpp          // 四相机测试程序
│   ├── binocular_camera_test.cpp // 双目相机测试程序
│   ├── gimbal_test.cpp           // 云台通信测试程序
│   ├── omniperception_test.cpp   // 全向感知测试程序
│   ├── planner_offline_replay.cpp// 规划器离线回放
│   ├── rbplan_smoke_test.cpp     // 规划器冒烟测试
│   ├── aim_armor_selection_test.cpp // 瞄准选板逻辑测试
│   ├── rune_*_test.cpp           // 打符测试（配置/运动拟合/位姿适配/解码/安全）
│   ├── uv_*_test.cpp             // UV观测测试（滤波、雅可比、回归、叠加渲染）
│   ├── tracker_offline_bench.cpp // 跟踪器离线跑分
│   ├── sp_example              // 往年示例程序
│   │   ├── camera_thread_test.cpp    // 相机线程测试程序
│   │   ├── cboard_test.cpp           // C板测试程序
│   │   ├── detector_video_test.cpp   // 识别器测试程序（视频）
│   │   ├── dm_test.cpp               // 达妙IMU测试程序
│   │   ├── fire_test.cpp             // 开火测试程序
│   │   ├── gimbal_response_test.cpp  // 云台响应测试程序
│   │   ├── handeye_test.cpp          // 手眼标定测试程序
│   │   ├── minimum_vision_system.cpp // 最小视觉系统测试程序
│   │   ├── multi_usbcamera_test.cpp  // 多USB摄像头测试程序
│   │   ├── planner_test.cpp          // 规划器测试程序（实车）
│   │   ├── planner_test_offline.cpp  // 规划器测试程序（离线）
│   │   ├── publish_test.cpp          // ROS发送测试程序
│   │   ├── subscribe_test.cpp        // ROS接收测试程序
│   │   ├── topic_loop_test.cpp       // ROS话题循环测试程序
│   │   ├── usbcamera_detect_test.cpp // 识别器测试程序（USB相机）
│   │   └── usbcamera_test.cpp        // USB相机测试程序
│   └── ...
└── tools          // 工具层，见 tools/README.md
    ├── crc.hpp                    // CRC校验
    ├── exiter.hpp                 // 退出检测
    ├── extended_kalman_filter.hpp // 扩展卡尔曼滤波器
    ├── fft.hpp                    // FFT周期运动分析
    ├── fps_solve.hpp              // 帧率统计
    ├── gimbal_calib.cpp           // 云台标定程序（独立可执行文件）
    ├── img_tools.hpp              // 图像处理工具
    ├── logger.hpp                 // 日志记录器
    ├── math_tools.hpp             // 数学工具
    ├── periodic_timer.hpp         // 绝对时刻周期调度器
    ├── pid.hpp                    // PID控制器
    ├── plotter.hpp                // 曲线图绘制工具
    ├── plotter_replay.cpp         // PlotJuggler记录回放程序
    ├── recorder.hpp               // 视频录制器
    ├── systemd_watchdog.hpp       // systemd看门狗心跳（见 自动启动.md）
    ├── thread_pool.hpp            // 线程池
    ├── thread_safe_queue.hpp      // 线程安全队列
    ├── trajectory.hpp             // 弹道解算
    ├── uv_overlay.hpp             // UV观测可视化叠加
    └── yaml.hpp                   // YAML配置文件解析器
```

## PlotJuggler 数据录制和回放

`Plotter` 默认实时发送并录制。构造函数的第一个参数用于选择是否录制：

```cpp
tools::Plotter recording_plotter(true);
tools::Plotter realtime_only_plotter(false);
```

数据默认保存到 `plot_records/时间戳.csv`，并保留同名 `.jsonl` 原始记录。也可以用第四个参数
指定文件，或在运行中调用 `start_recording(path)` / `stop_recording()` 开关录制。CSV 会在
`stop_recording()` 或程序正常退出时生成。

录制结束后，在 PlotJuggler 中选择 `Data -> Load data`，直接打开生成的 `.csv`，并选择第一列
`time` 作为时间轴，即可使用 PlotJuggler 自带的时间滑块回放，无需启动原视觉程序。

如果需要模拟原来的 UDP 实时输入，也可以执行：

```bash
./build/plotter_replay plot_records/2026-08-16_19-30-00_123456.csv
```

第二个参数可设置回放倍速，例如 `2.0` 为两倍速；后两个可选参数依次为目标 IP 和端口。


## 其他组在造车时需注意：

### 结构
#### 小电脑选型
- 在画云台前首先需要跟视觉确定好小电脑选型，目前实验室的小电脑在结构上可分为4种：
1. i5-8代intel nuc(尺寸相对较小)
2. 华硕nuc (三台华硕性能都不同，但是结构尺寸大小、接口位置都相同。 与8代结构尺寸相同，接口位置不同)
3. 雷神nuc(结构尺寸最大， usb3.0数量最多，与8代和华硕区别较大)
4. jetson orin nx (尺寸最小，重量最轻)
- 小电脑接口位置需要往外延伸3cm左右用于接线预留位置
#### 相机，镜头选型
- 实验室相机尺寸大差不差，但是各种型号的镜头尺寸不一，在结构画云台前一定要跟视觉确定相机和镜头选型，尽量做往最大尺寸作为参考、兼容其他镜头的相机保护壳。
#### 云台结构要求
1. 确保相机能安装在云台预留给相机的位置上，且不能存在干涉、镜头前方不能有遮挡物、相机朝向尽量与枪管朝向平行
2. 预留孔位，保证能看见相机背面的相机指示灯


### 电路
#### 小电脑供电
- 视觉需提供该机器人所需小电脑的供电电压大小，一般是19v 或 20v二选一
#### 相机线选型
- 配合电路进行相机线选型、以及相机线理线方案
- 串口板选型（ch340、ch343、cp210s）、走线

## 部署前准备
### 电控&结构
- 弹道散布较小
- 云台控制正常
- 相机和小电脑已安装
- 串口板已接线


### 小电脑环境配置依赖（部署前）
#### x86小电脑
- [HikRobot SDK](https://www.hikrobotics.com/cn2/source/support/software/MVS_STD_GML_V2.1.2_231116.zip)

- [daheng SDK](https://www.daheng-imaging.com/)
位置：下载中心 -> 软件下载 -> Galaxy_Linux_CN-EN_32bits/64bits
解压tar.gz后，用终端cd到文件夹路径，运行安装脚本 ./Galaxy_camera.run

- [OpenVINO](https://docs.openvino.ai/2024/get-started/install-openvino/install-openvino-archive-linux.html)
##### 命令行安装（对网络环境有要求）：
- 1. 导入Intel官方GPG公钥：
```bash
    curl -fsSL https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB | 
    sudo  gpg --dearmor -o /usr/share/keyrings/intel-openvino-archive-keyring.gpg
```
- 2. 添加统一的 OpenVINO APT 源（Ubuntu 22.04 Jammy）：
```bash
    echo "deb [signed-by=/usr/share/keyrings/intel-openvino-archive-keyring.gpg] https://apt.repos.intel.com/openvino      ubuntu22 main" | 
    sudo tee /etc/apt/sources.list.d/intel-openvino.list
```
- 3. 
```bash
    sudo apt update
    sudo apt install -y openvino-2024.6.0
    export OpenVINO_DIR=/usr/lib/cmake/openvino2024.6.0
    export LD_LIBRARY_PATH=/usr/lib/openvino-2024.6.0:$LD_LIBRARY_PATH
```


- 其余：
```bash
# 注意：jetson 系列不能装 libopencv-dev，需使用系统自带 opencv 或手动编译，
# 否则会覆盖原有版本、cmake 也找不到 libopencv-dev 安装
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \
    libfmt-dev \
    libeigen3-dev \
    libspdlog-dev \
    libyaml-cpp-dev \
    libusb-1.0-0-dev \
    libceres-dev \
    nlohmann-json3-dev \
    openssh-server \
    screen
```

#### 一键环境配置脚本

上面这些 apt / OpenVINO / 相机 SDK 检查和 `build` 目录配置可以交给脚本完成：

```bash
scripts/setup_env.sh                 # 装依赖 + 检查依赖 + 配置并编译 ./build
scripts/setup_env.sh --check-only    # 只检查依赖，不改动系统
```

常用选项：`--no-apt`、`--no-build`、`--build-dir DIR`、`--jobs N`、`--clean-build`、
`--with-openvino`（安装 OpenVINO 2024.6）、`--with-ros2-msgs`、`--configure-camera-sdk`
（把 `io/*/lib/<amd64|arm64>` 注册进 ldconfig）、`--force-opencv-apt`、`--no-persist`、
`--skip-checks`。脚本按 CPU 架构检查依赖（aarch64 → CUDA/TensorRT，x86_64 → OpenVINO），
与 `-DSP_VISION_BACKEND` 的自动选择规则一致。完整选项见 `scripts/setup_env.sh --help`。

#### jetson 系列小电脑 （arm架构）
to do list

### 相机调焦
#### 运行相机
更改配置文件
```yaml
camera_name: "hikrobot"    # 相机品牌 daheng / hikrobot / mindvision
camera_sn: "DA7188471"     # 相机序列号，相机唯一标识（需要与所使用相机一致；mindvision 不读该字段）
exposure_us: 2500          # 相机曝光
gain: 0.8                  # 相机增益，这里限定0~1之间（mindvision 不读该字段）
gamma: 0.5                 # 相机SDK gamma值，0~1之间，1代表不开伽马，越低gamma越强（海康分支不读该字段）
img_gamma: 0.5             # 软件亮度gamma，1.0 表示完全跳过软件gamma和配套降噪
vid_pid: "2ba2:4d55"       # 相机接口所对应的vid和pid，仅mindvision读取；大恒固定2ba2:4d55，海康固定2bdf:0001
flip: false                # 相机画面是否倒置（翻转），mindvision 不读该字段
mirror: false              # 相机画面是否镜像，mindvision 不读该字段
timestamp_offset_us: 1000  # 采集时间戳补偿量，单位us，由上层按需使用
```

`img_gamma` 不为 1 时还会在 YCrCb 空间做降噪，三个可选参数（默认值见 `configs/README.md`）：
`img_gamma_shadow_offset`、`img_gamma_luma_denoise_sigma`、`img_gamma_chroma_denoise_sigma`。

改完配置文件后，编译运行 `./build/camera_test` 观察画面。

## 部署步骤
### 1. 编译：
```bash
cmake -B build
make -C build/ -j`nproc`
# 或者直接按f5
```
推理后端不用手动选：CMake 会按 CPU 架构自动认（`aarch64`/Jetson → TensorRT，`x86_64` → OpenVINO），
对应的 CUDA/TensorRT/OpenVINO 没装会直接报错并提示装什么。确实要强制时：
```bash
cmake -B build -DSP_VISION_BACKEND=TENSORRT   # 或 OPENVINO / NONE(不编译检测后端)
```
TensorRT 后端的 CUDA 目标算力也会自动识别（查 `nvidia-smi` 的算力，如 8.9 → 89；比本机 nvcc
支持的还新时会自动降到 nvcc 支持的最大值并给出警告），要覆盖就加 `-DCMAKE_CUDA_ARCHITECTURES=89`。

### 2. 运行demo:
```bash
./build/auto_aim_test
# 或者点开进入./build/auto_aim_test后按f5
```

### -- 运行指定文件的main
点开想运行的文件后，直接按f5构建编译

### 3. 测试串口通信功能gimbal是否正常
运行./test/gimbal_test.cpp 检查配置文件，检查是否接收到电控数据。
配置文件串口通信信息：
```yaml
#####-----gimbal参数-----#####
com_port:                 # 串口板文件路径，按顺序尝试
  - "/dev/ttyACM0"
  - "/dev/ttyACM1"
# 陀螺仪是否装错, 编号012分别代表电控坐标系ypr，想换旋转轴只需要对编号进行互换即可，负号就是对当前值取负，交换完再取负
# 上车前记得对齐电控坐标系和济瞄惯性系
gimbal_y1: 1
gimbal_p2: -3
gimbal_r3: -2
# 济瞄的惯性系  z轴往上, x轴是枪管指向
#     x  z
#      \ |
#       \|
# y_ _ _ 0   z:yaw   x:roll   y:pitch
```
- 若通信成功，终端会打印出电控四元数转欧拉角的信息，检查通信流畅度，保证不能存在卡顿，注意电控陀螺仪坐标要与济瞄坐标系对齐
- 若通信不成功，检查串口助手是否接收到信息？检查帧头帧尾是否对齐？crc校验是否通过？如果确认没问题，可以换回老代码通信查看接收输出，如果也没数据，大概率是板子有问题或者电控有问题

### 4. 相机标定&手眼标定
calibration文件夹下有多个程序，最常用的是：\
calibrate_camera   相机标定程序  \
calibrate_handeye  固定标定板位姿的手眼标定程序  \
calibrate_robotworld_handeye  手眼标定程序（同时计算标定板位置）  \
capture            四元数采集&拍照程序


#### 运行 calibration/capture 进行数据采集
步骤：
1. 将assets/img_with_q的文件夹清空
2. 将标定板放置在相机视角大小的1/9位置上（根据焦距来决定），并将标定板向左或者右倾斜大概20°左右（增加相机标定数据对倾斜系数敏感），固定好位置后就不能再动（因为移动会影响手眼标定）
3. 遥控控制云台，对标定板进行拍照，拍照过程中要将整个标定板包含进去，确保相机视角全部都拍过（图像分九宫格依次拍照），保证拍照过程中底盘不能移动、云台不出现抖动（英雄云台抖动会出现带动底盘动的情况，建议先让电控把云台pid调好），大概拍个30张左右
#### 相机标定
拍照完成后数据会存放在assets/img_with_q，内容包括jpg图像和图像所对应的四元数信息，直接运行calibrate_camera，它会自动读取assets/img_with_q的信息，按空格键依次读取每个照片和对应四元数，运行结束之后终端输出相机标定信息
#### 手眼标定
将相机标定数据替换到配置文件上，运行calibrate_handeye进行手眼标定，就计算出来相机安装偏角

### 5. 运行自瞄测试程序
拿一辆靶车，运行aim_debug_mpc,小陀螺转动靶车，imshow观察重投影是否正确，如果不正确需要重新拍照标定

### 6. 时间戳对齐
由于相机快门时钟和陀螺仪时钟分别在视觉端和电控端，是两个不同的系统时钟，且相机快门时间会占据一定ms时长，所以需要对齐时间戳 \
找到代码里的（`src/rb_auto_aim_debug.cpp` 等 debug 程序；`rb_auto_standard.cpp` 等上场程序直接取 `gimbal.q(t)`，补偿量在相机配置的 `timestamp_offset_us` 里）
```cpp
auto q = gimbal.q(t - 3ms);
```
晃动云台观察静靶，观察速度和角速度直线是否跳变？如果出现跳变就更改3ms的数值，使其增大或者减小，直至晃动云台静靶的速度和角速度直线很小甚至没有，时间充裕建议单位换成us进行调整

### 7. 调静靶偏移量
1. 遥控开自瞄看是否锁敌，云台pid是否正常，如果不正常看一下视觉的识别帧率，再看一下电控是否埋雷
2. 锁敌打蛋观察是否击中装甲板中心位置，如果不在中心的话更改配置文件：
```yaml
yaw_offset: -0.8 # degree -2.5  yaw偏移量
pitch_offset: -1  # degree 2   pitch偏移量
```
配置文件找到以上参数位置并对其更改

### 8. 调planner和延迟
```yaml
#####-----aimer参数-----#####
# 远距离(target_h 超阈值)时的额外 pitch 补偿(度)；Planner 必须读到该键，缺失会直接退出
far_pitch_offset: 0
# 远距离且目标较高时的额外 pitch 补偿(度)；同上，必须存在
far_high_pitch_offset: 0
yaw_offset: 0                        # degree yaw偏移量
pitch_offset: 0                      # degree pitch偏移量
comming_angle: 60                    # degree
leaving_angle: 20                    # degree
decision_speed: 8                    # rad/s 区分高速/低速小陀螺的转速阈值
high_speed_delay_time: 0.030         # s  高速小陀螺发弹延迟
low_speed_delay_time: 0.015          # s  低速小陀螺&平移发弹延迟
gimbal_delay: 0.002                  # s  云台延迟补偿
shoot_offset: 2                      # 开火点在MPC预测时域中的下标，越界会直接抛异常退出

#####-----shooter参数-----#####
first_tolerance: 3                   # 近距离射击容差 degree
second_tolerance: 2                  # 远距离射击容差 degree
judge_distance: 2                    # 距离判断阈值 m
auto_fire: true                      # 是否由自瞄控制射击

#####-----planner-----#####
target_dist_error: -0.23             # 弹道距离补偿 m
target_h_error: 0.28                 # 弹道高度补偿 m
fire_thresh: 0.003                   # 原济瞄火控跟随火控阈值，更改影响火控频率和精度，已弃用
small_armor_tolerance: 0.12          # cm 小装甲板开火区间，一般是一个小装甲板的宽度
big_armor_tolerance: 0.22            # cm 大装甲板开火区间，一般是一个大装甲板的宽度
tower_and_base_armor_tolerance_: 0.10  # 26赛季前哨站和基地装甲板检测宽度
gimbal_control_delay: 0.08           # 云台跟随延迟 s
tower_pitch_prediction_time: 0.05    # 英雄前哨站预测

max_yaw_acc: 50                      # mpc最大yaw角速度约束，单位rad/s, 其中真正输出的角速度一般比50小，建议看曲线图进行判断
Q_yaw: [9e6, 0]                      # 不用管
R_yaw: [1]                           # 不用管

max_pitch_acc: 100                   # mpc最大pitch角速度约束，单位rad/s,参考yaw
Q_pitch: [9e6,0]                     # 不用管
R_pitch: [1]                         # 不用管
```
- high_speed_delay_time、low_speed_delay_time、decision_speed 在 aimer 和 planner 中都会读取
- far_pitch_offset、far_high_pitch_offset、target_dist_error、target_h_error、shoot_offset 缺失时 Planner 会直接退出，换配置文件时注意别漏

#### 调云台跟随延迟 gimbal_control_delay
云台跟随延迟指的是视觉发送信息到电控到电控将电机转到目标角度所需时间，如果没有动力学的话需要调此值，如果上动力学的话需要将此值设成0。

#### 看曲线图
主要看gimbal_yaw和target_yaw、gimbal_pitch和target_pitch两者之间的重合程度，不上动力学的话需要调gimbal_control_delay使得在靶车匀速移动0.7s后gimbal_yaw和target_yaw两者曲线有较高重合度，如果上动力学需要电控调整滑膜，视觉不用管

#### 调发弹延迟
将small_armor_tolerance调小（大概0.05）之后测平移靶打蛋，观察发出去的子弹是否命中装甲板中心，如果不在中心，观察是超调还是滞后来调节low_speed_delay_time，调完之后从低速到高速测不通转速的小陀螺来观察命中率，高速小陀螺调high_speed_delay_time，统计各转速的小陀螺、平移小拓宽、单纯平移打击靶车的命中率。再细调一波直至命中率达到最高
