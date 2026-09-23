# TensorRT YOLO

## 主要类和函数

- `TensorRTYolo0708`、`TensorRTYolo0526`：实现统一 `YOLOBase` 接口，把 TensorRT 结果转换成 `Armor`。
- `detect(img)`：同步识别一帧图像。
- `detect(YOLOFrameData)`：提交异步任务并返回已经完成的帧。
- `TensorrtInferEngine::infer()`：在同步执行槽上完成预处理、推理和输出拷贝。
- `async_enabled_infer()`：使用任务队列、空闲显存槽和结果队列运行异步推理。
- `decode_outputs()`：执行 sigmoid/softmax、置信度过滤、颜色过滤和 NMS。
- `TRTExecutionSlot<T>`：管理每个推理任务独立的执行上下文、CUDA stream、输入输出显存和固定页内存。
- `trt_0708_kernel.cu`、`trt_0526_kernel.cu`：GPU 图像预处理及模型相关 CUDA kernel。

## 原理

TensorRT 从序列化 `.engine` 文件恢复优化后的网络。CPU 准备帧元数据，CUDA kernel 把图像转换为模型输入，TensorRT 在独立 stream 上执行推理，再将结果拷回 CPU 做解码和 NMS。

异步模式复用一组预分配执行槽，避免每帧重新申请显存。每个并行任务必须拥有独立 `IExecutionContext` 和 CUDA stream，否则不同帧会互相覆盖绑定地址和输出。

`0708` 和 `0526` 模型输出布局、精度类型及类别字段不同，不应共用解码函数。修改 engine 或 CUDA kernel 后，需要重新编译（后端按架构自动选，强制 TensorRT 用 `-DSP_VISION_BACKEND=TENSORRT`）。
