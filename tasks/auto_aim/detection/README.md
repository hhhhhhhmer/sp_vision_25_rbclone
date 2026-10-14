# Detection

## 作用

输入一帧 OpenCV BGR 图像，输出统一的 `std::list<Armor>`。识别层只处理图像信息，不负责三维解算和目标状态预测。

## 主要类和函数

### 神经网络入口

- `YOLO(config_path, debug)`：读取 `yolo_name`，根据编译选项创建 OpenVINO 或 TensorRT 后端。
- `YOLO::detect(img, frame_count)`：同步检测入口，返回当前图像的装甲板列表。
- `YOLO::detect(YOLOFrameData, frame_count)`：异步入口，保留图像、姿态和时间戳元数据。
- `YOLO::postprocess()`：把已有网络输出交给具体后端解析，供多线程检测器复用。
- `YOLOFrameData`：异步流程中的一帧数据，包含图像、时间戳、云台四元数和检测结果。

### 传统视觉与分类

- `Detector::detect(bgr_img)`：灰度化、二值化、提取灯条、两两配对、数字分类并去除重复装甲板。
- `Detector::detect(armor, bgr_img)`：在已有候选框内重新提取灯条。
- `Classifier::classify(armor)`：将装甲板数字区域缩放到 `32x32`，经 ONNX 网络和 softmax 得到编号及置信度。
- `Classifier::ovclassify(armor)`：OpenVINO 版同类数字分类，目前没有调用点（保留接口，实际调用的是 `classify()`）。
- `resolve_roi(configured, image_size)`：把支持 `-1` 宽高的 ROI 配置解析为合法图像区域，并检查越界。

### 其他识别器

- `rv_detector.*`、`rv_number.*`：基于灯条几何的另一套装甲板检测与数字分类实现，位于 `rv_aim` 命名空间（与 `auto_aim::Detector` 是两个不同的类）；`NumberClassifier` 由 YAML 的 `number_classifier` 节点构造，其中 `ignore_classes` 当前未被读取。
- `NumberClassifier::extractNumbers()`：从装甲板区域提取数字图案。
- `NumberClassifier::classify()`：使用 OpenCV DNN 对候选装甲板批量分类。

`yolo_name` 的可选值随后端不同：OpenVINO 支持 `yolov5`、`ov_0526`、`yolov8`、`yolo11`；TensorRT 支持 `trt_0526`、`trt_0708`。未知取值会抛异常。

## 原理

传统视觉依赖“高亮灯条具有稳定颜色和几何比例”的假设：先找轮廓，再按长度、角度、平行性和间距组合装甲板，最后单独识别数字。YOLO 后端则直接回归检测框、类别和四个关键点，再经过置信度筛选、NMS、关键点排序和几何检查生成 `Armor`。

无论使用哪种方法，输出的四角点顺序、颜色编码和类别枚举必须一致，因为 `Solver` 和 `Tracker` 只依赖统一的 `Armor`。

模型输出格式修改时先看具体后端的 `parse()` 或 `postprocess()`；模型路径、输入尺寸、置信度、NMS 阈值和 ROI 通常来自 YAML。
