#ifndef AUTO_AIM__TENSORRT_YOLO_HPP
#define AUTO_AIM__TENSORRT_YOLO_HPP

#include <list>
#include <string>
#include <opencv2/opencv.hpp>

#include <vector>
#include <NvInfer.h>
#include <cuda_runtime.h>
#include <iostream>
#include <chrono>
#include <cmath>

#include "../armor.hpp"
#include "../yolo.hpp"
#include "tools/thread_safe_queue.hpp"

using namespace cv;
using namespace std;

namespace auto_aim
{
struct Object {
    cv::Rect_<float> rect;
    float landmarks[8];
    int label;
    float prob;
    int color;      // 模型输出: blue:1 , red:0
    double length;
    double width;
    double ratio;
};

struct TRTFrameData : public YOLOFrameData {
    int frame_id;
    cudaStream_t stream;
    
    // 独占资源
    unsigned char* d_img = nullptr;  // 每帧独占的输入图像显存
    size_t d_img_size = 0;
    void* input_device = nullptr;    
    void* output_device = nullptr;   
    float* output_host = nullptr;    // 每帧独占的页锁定内存输出

    std::vector<Object> detected_objects; // 存放该帧的解析结果

    void setTRTFrameData(const YOLOFrameData& f) {
        this->frame = f.frame;
        this->gimbal_q = f.gimbal_q;
        this->timestamp = f.timestamp;
        this->armors = f.armors;
        this->detect_color = f.detect_color;
        this->is_empty = false; 
    }
};

class TensorrtInferEngine {
public:
    vector<Object> objects;
    vector<Object> tmp_objects;
    const int IMAGE_HEIGHT = 640;
    const int IMAGE_WIDTH = 640;
    float* d_output_host = nullptr;
    cudaEvent_t preprocess_done;   // 预处理完成事件
    cudaEvent_t inference_done;    // 推理完成事件
    cudaEvent_t copy_done;         // 拷贝完成事件

    TensorrtInferEngine(const string& engine_path, const string& device = "cuda:0");
    ~TensorrtInferEngine();
    void infer(Mat img, int detect_color);
    bool async_enabled_infer(const cv::Mat& bgr_img, const YOLOFrameData& frame_info, int frame_count, int detect_color, std::vector<Object>& out_objects, YOLOFrameData& out_frame_data);

    std::vector<Object> decode_outputs(const float* output_ptr, int detect_color);
private:
    nvinfer1::ICudaEngine* engine = nullptr;
    nvinfer1::IExecutionContext* context = nullptr;
    cudaStream_t stream = nullptr;
    tools::ThreadSafeQueue<TRTFrameData> task_queue; // 待推理队列
    tools::ThreadSafeQueue<TRTFrameData> free_buffer_queue; // 空闲显存池队列
    tools::ThreadSafeQueue<TRTFrameData> result_queue;      // 结果队列

    void* input_device = nullptr;
    void* output_device = nullptr;
    size_t input_size = 0;
    size_t output_size = 0;
    string input_name;
    string output_name;

    // GPU 内存复用变量，避免每帧开辟显存导致的严重掉帧
    unsigned char* d_img = nullptr;
    size_t d_img_size = 0;

    std::thread infer_work_thread;
    bool infer_work_thread_is_running = true;
    int in_flight_count = 0; // 记录当前在 GPU 中飞行的帧数（用于流水线控制）
    void infer_workerLoop();

    void loadEngine(const string& engine_path);
    double sigmoid(double x) {
        if (x > 0) return 1.0 / (1.0 + exp(-x));
        else return exp(x) / (1.0 + exp(x));
    }
    static void softmax(const float* input, float* output, int len);
};

class TensorRTYolo0708 : public YOLOBase
{
public:
    TensorRTYolo0708(const std::string& config_path, bool debug = true);
    ~TensorRTYolo0708() = default;

    std::list<Armor> detect(const cv::Mat& img, int frame_count = -1) override;

    // TUDO:推理线程
    YOLOFrameData detect(YOLOFrameData frame_data, int frame_count = -1) override;

    std::list<Armor> postprocess(double scale, cv::Mat& output,
                                 const cv::Mat& bgr_img, int frame_count) override;

private:
    std::unique_ptr<TensorrtInferEngine> engine_;
    bool debug_;
    bool use_roi_;
    cv::Rect roi_;
    cv::Point2f offset_;

    double min_confidence_;
    int detect_color_;          // -1: all, 0: red, 1: blue

    // 辅助函数
    std::list<Armor> convertToArmors(const cv::Mat& raw_img, int frame_count);
    void sortKeypoints(std::vector<cv::Point2f>& pts);
    bool checkName(const Armor& armor) const;
    bool checkType(const Armor& armor) const;
    ArmorType getType(const Armor& armor) const;
    void drawDetections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
};

} // namespace auto_aim

#endif // AUTO_AIM__TENSORRT_YOLO_HPP
