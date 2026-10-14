# ROS2

## 主要类和函数

- `ROS2::publish(target_pos)`：通过 `Publish2Nav` 发布目标四维位置。
- `subscribe_enemy_status()`：取得订阅线程保存的最新敌方状态。
- `subscribe_autoaim_target()`：取得最新导航自瞄目标信息。
- `Publish2Nav()` / `Subscribe2Nav()`：构造函数里创建发布者/订阅者并绑定回调。
- `Publish2Nav::start()` / `Subscribe2Nav::start()`：只负责进入 spin 循环，不创建话题端点。
- `Publish2Nav::send_data()`：把 Eigen 数据编码成 ROS 消息并发布。
- `enemy_status_callback()`、`autoaim_target_callback()`：接收消息并写入线程安全队列。

## 原理

发布和订阅节点在独立 ROS2 spin 线程中运行，视觉主循环只调用简单同步接口。订阅回调把消息放入队列，避免直接从 ROS 回调线程修改视觉算法状态。

这是可选模块；只有 `ament_cmake`、`rclcpp`、消息包和类型支持全部存在时才会加入 `io` 目标，否则 CMake 会跳过它。
