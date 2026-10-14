# Multithread Detection

## 主要类型和函数

- `DetectionFrame`：保存帧编号、原图、时间戳、姿态和检测结果。
- `create_detectors(config_path, number, debug)`：创建多个相互独立的 `YOLO` 实例，供线程池并行使用。
- `OrderedDetectionQueue::enqueue()`：接收可能乱序完成的检测结果，按帧编号暂存并恢复顺序。
- `dequeue()`：阻塞取得下一帧有序结果。
- `try_dequeue()`：非阻塞尝试取得结果。
- `size()`：返回主队列与乱序缓冲区的总帧数。
- `MultiThreadDetector::push(img, t)`：创建 OpenVINO `InferRequest` 并异步提交一帧。
- `pop()`：等待队首请求完成，返回装甲板和原始时间戳。
- `debug_pop()`：额外返回原图，便于绘制调试结果。

## 原理

多线程或异步推理会让后提交的帧先完成。`OrderedDetectionQueue` 用 `frame.id` 把提前完成的结果放入缓冲区，只有连续编号到达时才进入主队列，从而保证 Tracker 按时间顺序收到观测。

`MultiThreadDetector` 把预处理后的图像和对应 `InferRequest` 一起入队，`pop()` 再等待并后处理，因此时间戳始终属于原始图像，而不是取结果的时刻。它复用 `YOLO::postprocess()` 的解码路径，
目前暂不支持 `yolov8` 和 ROI。

先理解单线程 `YOLO::detect()`，再修改这里。需要重点检查队列满、旧帧丢弃、程序退出和结果乱序。
