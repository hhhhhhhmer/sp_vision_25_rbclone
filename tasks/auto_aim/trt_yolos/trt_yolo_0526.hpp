#ifndef AUTO_AIM__TRT_YOLO_0526_HPP
#define AUTO_AIM__TRT_YOLO_0526_HPP

#include "../yolo.hpp"
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cuda_fp16.h>
#include "tools/thread_safe_queue.hpp"

namespace auto_aim {

// 检测结果结构
struct Object0526 {
    cv::Rect_<float> rect;
    float landmarks[8];
    int digit_id;      // 数字类别 0~8
    int color_id;      // 颜色类别 (0=红, 1=蓝, 按模型定义)
    float prob;
    double length, width, ratio;
};

// 帧数据（含 GPU 资源）
struct TRTFrameData0526 : public YOLOFrameData {
    int frame_id;
    cudaStream_t stream;

    unsigned char* d_img = nullptr;
    size_t d_img_size = 0;
    half* input_device = nullptr;
    float* output_device = nullptr;
    float* output_host = nullptr;

    std::vector<Object0526> detected_objects;

    void setTRTFrameData(const YOLOFrameData& f) {
        this->frame = f.frame;
        this->gimbal_q = f.gimbal_q;
        this->timestamp = f.timestamp;
        this->armors = f.armors;
        this->detect_color = f.detect_color;
        this->is_empty = false;
    }
};

// 异步推理引擎
class TensorrtInferEngine0526 {
public:
    static constexpr int IMAGE_WIDTH  = 640;
    static constexpr int IMAGE_HEIGHT = 640;

    TensorrtInferEngine0526(const std::string& engine_path, const std::string& device = "cuda:0");
    ~TensorrtInferEngine0526();

    void infer(const cv::Mat& bgr_img,
                                    int frame_count,
                                    int detect_color,
                                    std::vector<Object0526>& out_objects,
                                    YOLOFrameData& out_frame_data);
                            
    bool async_enabled_infer(const cv::Mat& bgr_img,
                             const YOLOFrameData& frame_info,
                             int frame_count,
                             int detect_color,
                             std::vector<Object0526>& out_objects,
                             YOLOFrameData& out_frame_data);

private:
    nvinfer1::ICudaEngine* engine_ = nullptr;
    nvinfer1::IExecutionContext* context_ = nullptr;

    tools::ThreadSafeQueue<TRTFrameData0526> task_queue_;
    tools::ThreadSafeQueue<TRTFrameData0526> free_buffer_queue_;
    tools::ThreadSafeQueue<TRTFrameData0526> result_queue_;

    std::thread worker_thread_;
    bool worker_running_ = true;
    int in_flight_count_ = 0;

    size_t input_size_ = 0, output_size_ = 0;
    std::string input_name_, output_name_;

    void workerLoop();
    std::vector<Object0526> decode_outputs(const float* output_ptr, int detect_color);

    static float sigmoid(float x) { return 1.0f / (1.0f + std::exp(-x)); }
    static void softmax(const float* input, float* output, int len);

    float conf_threshold_ = 0.65f;
    float nms_threshold_ = 0.45f;
};

// YOLO 封装类
class TensorRTYolo0526 : public YOLOBase {
public:
    explicit TensorRTYolo0526(const std::string& config_path, bool debug = true);
    ~TensorRTYolo0526() = default;

    std::list<Armor> detect(const cv::Mat& img, int frame_count = -1) override;
    YOLOFrameData detect(YOLOFrameData frame_data, int frame_count = -1) override;
    std::list<Armor> postprocess(double scale, cv::Mat& output,
                                 const cv::Mat& bgr_img, int frame_count) override;

private:
    std::unique_ptr<TensorrtInferEngine0526> engine_;
    bool debug_;
    int detect_color_;
    double min_confidence_;

    std::list<Armor> convertToArmors(const std::vector<Object0526>& objects,
                                     const cv::Mat& img,
                                     int frame_count);
    void sortKeypoints(std::vector<cv::Point2f>& pts);
    bool checkName(const Armor& armor) const;
    bool checkType(const Armor& armor) const;
    void drawDetections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
};

} // namespace auto_aim

#endif
