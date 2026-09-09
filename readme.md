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
├── configs        // 每台机器人的YAML配置文件
│   └── ...
├── io             // 硬件抽象层，见3.4软件架构
│   └── ...
├── src            // 应用层，调试/上场代码
|   ├── 1.cpp
|   ├── auto_aim_debug_mpc.cpp          //原济瞄aim mpc debug，已弃用
|   ├── auto_buff_debug.cpp             //原济瞄buff debug
|   ├── auto_buff_debug_mpc.cpp         //原济瞄buff mpc debug
|   ├── hero.cpp                        //26赛季英雄专用
|   ├── mt_auto_aim_debug.cpp           //没用
|   ├── mt_standard.cpp                 //没用
|   ├── rb_auto_aim_debug.cpp           //26赛季步兵、哨兵通用aim mpc debug
|   ├── rb_binocular_aim_debug.cpp      //26赛季双目自瞄debug
|   ├── sentry_binocular_aim_mpc.cpp    //26赛季新烧饼双目视觉、全向感知集大乘
|   ├── sentry_bp.cpp                   //没用
|   ├── sentry.cpp                      //没用
|   ├── sentry_debug.cpp                //没用
|   ├── sentry_multithread.cpp          //没用
|   ├── standard.cpp                    //没用
|   ├── standard_mpc.cpp                //auto_buff_debug_mpc没imshow版本
|   ├── uav.cpp                         //没用
|   └── uav_debug.cpp                   //没用
|
├── tasks          // 功能层，见3.4软件架构
│   ├── auto_aim       // 自瞄相关算法实现
│   │   └── ...
│   ├── auto_buff      // 打符相关算法实现
│   │   └── ...
│   └── omniperception // 全向感知相关算法实现
│   │   └── ...
├── tests    //用于测试的代码
│   ├── auto_aim_test.cpp         // 自瞄录制视频测试程序
│   ├── auto_buff_test.cpp        // 打符录制视频测试程序
│   ├── camera_detect_test.cpp    // 识别器测试程序（工业相机）
│   ├── camera_test.cpp           // 相机测试程序
│   ├── camera_thread_test.cpp    // 相机线程测试程序
│   ├── cboard_test.cpp           // C板测试程序
│   ├── detector_video_test.cpp   // 识别器测试程序（视频）
│   ├── dm_test.cpp               // 达妙IMU测试程序
│   ├── fire_test.cpp             // 开火测试程序
│   ├── gimbal_response_test.cpp  // 云台响应测试程序
│   ├── gimbal_test.cpp           // 云台通信测试程序
│   ├── handeye_test.cpp          // 手眼标定测试程序
│   ├── minimum_vision_system.cpp // 最小视觉系统测试程序
│   ├── multi_usbcamera_test.cpp  // 多USB摄像头测试程序
│   ├── planner_test_offline.cpp  // 规划器测试程序（离线）
│   ├── planner_test.cpp          // 规划器测试程序（实车）
│   ├── publish_test.cpp          // ROS发送测试程序
│   ├── subscribe_test.cpp        // ROS接收测试程序
│   ├── topic_loop_test.cpp       // ROS话题循环测试程序
│   ├── usbcamera_detect_test.cpp // 识别器测试程序（USB相机）
│   ├── usbcamera_test.cpp        // USB相机测试程序
│   └── ...
└── tools          // 工具层，见3.4软件架构
    ├── crc.hpp                    // CRC校验
    ├── exiter.hpp                 // 退出检测
    ├── extended_kalman_filter.hpp // 扩展卡尔曼滤波器
    ├── img_tools.hpp              // 图像处理工具
    ├── logger.hpp                 // 日志记录器
    ├── math_tools.hpp             // 数学工具
    ├── plotter.hpp                // 曲线图绘制工具
    ├── recorder.hpp               // 视频录制器
    ├── thread_safe_queue.hpp      // 线程安全队列
    ├── trajectory.hpp             // 弹道解算
    ├── yaml.hpp                   // YAML配置文件解析器
    └── ...

```


## 其他组在造车时需注意：

### 结构
#### 小电脑选型
- 在画云台前首先需要跟视觉确定好小电脑选型，目前实验室的小电脑在结构上可分为4种：
1. i5-8代intel nuc(尺寸相对较小)
2. 华硕nuc (三台华硕性能都不同，但是结构尺寸大小、接口位置都相同。 与8代结构尺寸相同，接口位置不同)
3. 雷神nuc(结构尺寸最大， usb3.0数量最多，与8代和华硕区别较大)
4. jetson orin nano (尺寸最小，重量最轻)
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
sudo apt install -y \
    git \
    g++ \
    cmake \
    can-utils \
    libopencv-dev \  # jetson系列不能运行此命令，需使用自带opencv或者手动编译，否则会覆盖原有版本,cmake找不到libopencv-dev安装
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
#### jetson 系列小电脑 （arm架构）
to do list

### 相机调焦
#### 运行相机
更改配置文件
```yaml
camera_name: "daheng"      # 相机品牌 daheng 或者hikrobot
camera_sn: "KE0210030295"  # 相机序列号，相机唯一标识（需要与所使用相机一致）
exposure_us: 4000          # 相机曝光
gain: 0.5                  # 相机增益，这里限定0~1之间
gamma: 0.5                 # 相机gamma值，0~1之间，1代表不开伽马，越低gamma越强
img_gamma: 0.5
vid_pid: "2ba2:4d55"       # 相机接口所对应的vid和pid，一般同一个相机品牌vid_pid都相同，大恒是2ba2:4d55， 海康是2bdf:0001
flip: false                # 相机画面是否倒置（翻转）
mirror: false              # 相机画面是否镜像
```
一栏后，编译运行./build/camera_test

## 部署步骤
### 1. 编译：
```bash
cmake -B build
make -C build/ -j`nproc`
# 或者直接按f5
```

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
# 济瞄的惯性系  z轴往上, x轴反方向是枪管指向
#        z
#        |
#        |
# y_ _ _ 0   z:yaw   x:roll   y:pitch
          \
           \
            \x
```
- 若通信成功，终端会打印出电控四元数转欧拉角的信息，检查通信流畅度，保证不能存在卡顿，注意电控陀螺仪坐标要与济瞄坐标系对齐
- 若通信不成功，检查串口助手是否接收到信息？检查帧头帧尾是否对齐？crc校验是否通过？如果确认没问题，可以换回老代码通信查看接收输出，如果也没数据，大概率是板子有问题或者电控有问题

### 4. 相机标定&手眼标定
calibration文件夹下有三个文件需要用到：\
calibrate_camera   相机标定程序  \
calibrate_handeye  固定标定板位姿的手眼标定程序  \
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
找到代码里的
```cpp
auto q = gimbal.q(timestamp - 3ms);
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
high_speed_delay_time: 0.09          # s  高速小陀螺发弹延迟
low_speed_delay_time: 0.09           # s  低速小陀螺&平移发弹延迟

... 

#####-----planner-----#####
fire_thresh: 0.003                   # 原济瞄火控跟随火控阈值，更改影响火控频率和精度，已弃用
small_armor_tolerance: 0.12          # cm 小装甲板开火区间，一般是一个小装甲板的宽度
big_armor_tolerance: 0.22            # cm 大装甲板开火区间，一般是一个大装甲板的宽度
tower_and_base_armor_tolerance_: 0.10  # 26赛季前哨站和基地装甲板检测宽度
gimbal_control_delay: 0.08           # 云台跟随延迟 s
tower_pitch_prediction_time: 0.05    # 英雄前哨站预测

max_yaw_acc: 50                      # mpc最大yaw角速度约束，单位rad/s, 其中真正输出的角速度一般比50小，建议看曲线图进行判断
Q_yaw: [9e6, 0]                      # 不用管
R_yaw: [1]                           # 不用管

max_pitch_acc: 100                   # mpc最大yaw角速度约束，单位rad/s,参考yaw
Q_pitch: [9e6,0]                     # 不用管
R_pitch: [1]                         # 不用管
```
- high_speed_delay_time和low_speed_delay_time在planner也有用到

#### 调云台跟随延迟 gimbal_control_delay
云台跟随延迟指的是视觉发送信息到电控到电控将电机转到目标角度所需时间，如果没有动力学的话需要调此值，如果上动力学的话需要将此值设成0。

#### 看曲线图
主要看gimbal_yaw和target_yaw、gimbal_pitch和target_pitch两者之间的重合程度，不上动力学的话需要调gimbal_control_delay使得在靶车匀速移动0.7s后gimbal_yaw和target_yaw两者曲线有较高重合度，如果上动力学需要电控调整滑膜，视觉不用管

#### 调发弹延迟
将small_armor_tolerance调小（大概0.05）之后测平移靶打蛋，观察发出去的子弹是否命中装甲板中心，如果不在中心，观察是超调还是滞后来调节low_speed_delay_time，调完之后从低速到高速测不通转速的小陀螺来观察命中率，高速小陀螺调high_speed_delay_time，统计各转速的小陀螺、平移小拓宽、单纯平移打击靶车的命中率。再细调一波直至命中率达到最高
