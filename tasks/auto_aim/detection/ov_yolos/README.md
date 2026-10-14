# OpenVINO YOLO

## 主要类和函数

- `YOLOV5`、`YOLOV8`、`YOLO11`：三种模型格式对应的 OpenVINO 实现，都继承 `YOLOBase`。
- `detect(bgr_img, frame_count)`：完成缩放补边、同步推理和后处理。
- `detect(YOLOFrameData, frame_count)`：使用异步槽提交帧；流水线尚未填满时会返回空帧。
- `parse(scale, output, image, frame_count)`：解释网络张量、恢复原图坐标、执行 NMS 并构造 `Armor`。
- `postprocess()`：公开后处理入口，供外部异步推理代码调用。
- `check_name()`、`check_type()`：过滤不合法类别和尺寸组合。
- `draw_detections()`：调试模式下绘制识别结果（调用点由 `debug_` 把守）。
- `save()`：把可疑样本落盘，但三个后端里的调用点目前都被注释掉，实际不会保存。
- `yolo_name: ov_0526` 在 OpenVINO 下同样映射到 `YOLOV5` 实现。

### `OpenVINOAsyncPipeline`

- `init(compiled_model, width, height)`：为两个推理槽创建独立 `InferRequest`。
- `detect(image, frame_data, frame_count, parser)`：提交新帧，同时取回最早完成的旧帧。

## 原理

输入图像按比例缩放后放入固定尺寸画布，避免直接拉伸导致几何变形。模型输出经过置信度筛选和 NMS，删除同一目标上的重复框；关键点再映射回原图坐标并按固定顺序排列。

异步流水线使用两个独立槽交叠 CPU 预处理和设备推理。输出会比输入延后一帧左右，因此必须随帧保存原图、姿态和时间戳，不能拿当前时刻的数据匹配旧检测结果。

修改模型时需要同时核对输入布局、颜色格式、输出张量维度、类别编码和关键点排列。
