#include "trt_yolo_0526.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include "tools/logger.hpp"
#include "tools/img_tools.hpp"

#define IS_ASYNC true

extern "C" void launchPreprocess(const unsigned char* src, half* dst,
                                 int src_width, int src_height, int src_step,
                                 int dst_width, int dst_height,
                                 cudaStream_t stream);

namespace auto_aim {

// ---------- Logger（匿名命名空间避免重复定义） ----------
namespace {
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            std::cout << "[TensorRT 0526] " << msg;
    }
} gLogger;
} // namespace

// ---------- TensorrtInferEngine0526 实现 ----------
TensorrtInferEngine0526::TensorrtInferEngine0526(const std::string& engine_path,
                                                 const std::string& device)
    : task_queue_(3), free_buffer_queue_(3), result_queue_(3) {
    std::ifstream file(engine_path, std::ios::binary);
    if (!file) throw std::runtime_error("Cannot open engine file: " + engine_path);
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(size);
    file.read(data.data(), size);
    file.close();

    auto runtime = nvinfer1::createInferRuntime(gLogger);
    engine_ = runtime->deserializeCudaEngine(data.data(), size);
    delete runtime;
    if (!engine_) throw std::runtime_error("Failed to deserialize engine");

    context_ = engine_->createExecutionContext();
    if (!context_) throw std::runtime_error("Failed to create context");

    input_name_ = engine_->getIOTensorName(0);
    output_name_ = engine_->getIOTensorName(1);
    auto input_dims = engine_->getTensorShape(input_name_.c_str());
    auto output_dims = engine_->getTensorShape(output_name_.c_str());

    input_size_ = 1 * input_dims.d[1] * input_dims.d[2] * input_dims.d[3] * sizeof(half);
    output_size_ = 1 * output_dims.d[1] * output_dims.d[2] * sizeof(float);


    auto init_stream = [this](){
        TRTFrameData0526 task;
        task.frame_id = -1;
        cudaStreamCreate(&task.stream);
        cudaMalloc(&task.input_device, input_size_);
        cudaMalloc(&task.output_device, output_size_);
        cudaHostAlloc(&task.output_host, output_size_, cudaHostAllocDefault);
        free_buffer_queue_.push(task);
    };
    if(IS_ASYNC){
        const int POOL_SIZE = 3;
        for (int i = 0; i < POOL_SIZE; ++i) {
            init_stream();
        }        
    }else{
        init_stream();
    }
    worker_running_ = true;
    worker_thread_ = std::thread(&TensorrtInferEngine0526::workerLoop, this);
}

TensorrtInferEngine0526::~TensorrtInferEngine0526() {
    worker_running_ = false;
    TRTFrameData0526 poison;
    poison.frame_id = -1;
    task_queue_.push(poison);
    if (worker_thread_.joinable()) worker_thread_.join();

    auto free_task = [](TRTFrameData0526& t) {
        if (t.stream) cudaStreamDestroy(t.stream);
        if (t.input_device) cudaFree(t.input_device);
        if (t.output_device) cudaFree(t.output_device);
        if (t.output_host) cudaFreeHost(t.output_host);
        if (t.d_img) cudaFree(t.d_img);
    };
    TRTFrameData0526 tmp;
    while (free_buffer_queue_.try_pop(tmp)) free_task(tmp);
    while (task_queue_.try_pop(tmp))       free_task(tmp);
    while (result_queue_.try_pop(tmp))     free_task(tmp);

    delete context_;
    delete engine_;
}

void TensorrtInferEngine0526::workerLoop() {
    while (worker_running_) {
        TRTFrameData0526 task = task_queue_.pop();
        if (task.frame_id == -1) break;

        context_->setTensorAddress(input_name_.c_str(), task.input_device);
        context_->setTensorAddress(output_name_.c_str(), task.output_device);
        context_->enqueueV3(task.stream);
        cudaMemcpyAsync(task.output_host, task.output_device,
                        output_size_, cudaMemcpyDeviceToHost, task.stream);
        cudaStreamSynchronize(task.stream);
        result_queue_.push(task);
    }
}

void TensorrtInferEngine0526::infer(const cv::Mat& bgr_img,
                                    int frame_count,
                                    int detect_color,
                                    std::vector<Object0526>& out_objects,
                                    YOLOFrameData& out_frame_data) {
    TRTFrameData0526 task = free_buffer_queue_.pop();
    task.setTRTFrameData({bgr_img});
    task.frame_id = frame_count;
    task.detect_color = detect_color;

    size_t needed = bgr_img.total() * bgr_img.elemSize();
    if (task.d_img_size < needed) {
        if (task.d_img) cudaFree(task.d_img);
        cudaMalloc(&task.d_img, needed);
        task.d_img_size = needed;
    }
    cudaMemcpyAsync(task.d_img, bgr_img.data, needed,
                    cudaMemcpyHostToDevice, task.stream);
    launchPreprocess(task.d_img, task.input_device,
                     bgr_img.cols, bgr_img.rows, bgr_img.step,
                     IMAGE_WIDTH, IMAGE_HEIGHT, task.stream);

    context_->setTensorAddress(input_name_.c_str(), task.input_device);
    context_->setTensorAddress(output_name_.c_str(), task.output_device);
    context_->enqueueV3(task.stream);
    cudaMemcpyAsync(task.output_host, task.output_device,
                    output_size_, cudaMemcpyDeviceToHost, task.stream);
    cudaStreamSynchronize(task.stream);

    out_objects = decode_outputs(task.output_host, task.detect_color);
    out_frame_data = task;  
    // task = TRTFrameData0526();
    free_buffer_queue_.push(task);

}

bool TensorrtInferEngine0526::async_enabled_infer(const cv::Mat& bgr_img,
                                                  const YOLOFrameData& frame_info,
                                                  int frame_count,
                                                  int detect_color,
                                                  std::vector<Object0526>& out_objects,
                                                  YOLOFrameData& out_frame_data) {
    TRTFrameData0526 task = free_buffer_queue_.pop();
    task.setTRTFrameData(frame_info);
    task.frame_id = frame_count;
    task.detect_color = detect_color;

    size_t needed = bgr_img.total() * bgr_img.elemSize();
    if (task.d_img_size < needed) {
        if (task.d_img) cudaFree(task.d_img);
        cudaMalloc(&task.d_img, needed);
        task.d_img_size = needed;
    }
    cudaMemcpyAsync(task.d_img, bgr_img.data, needed,
                    cudaMemcpyHostToDevice, task.stream);
    launchPreprocess(task.d_img, task.input_device,
                     bgr_img.cols, bgr_img.rows, bgr_img.step,
                     IMAGE_WIDTH, IMAGE_HEIGHT, task.stream);

    task_queue_.push(task);
    in_flight_count_++;

    const int MAX_IN_FLIGHT = 2;
    if (in_flight_count_ >= MAX_IN_FLIGHT) {
        TRTFrameData0526 finished = result_queue_.pop();
        in_flight_count_--;

        out_objects = decode_outputs(finished.output_host, finished.detect_color);
        out_frame_data = finished;
        out_frame_data.is_empty = false;

        free_buffer_queue_.push(finished);
        return true;
    }
    return false;
}

std::vector<Object0526> TensorrtInferEngine0526::decode_outputs(const float* output_ptr,
                                                                int detect_color) {
    std::vector<Object0526> objects;
    objects.reserve(1000);
    const int rows = 25200, cols = 22;
    float color_probs[4], digit_probs[9];

    for (int i = 0; i < rows; ++i) {
        const float* ptr = output_ptr + i * cols;
        float obj_conf = sigmoid(ptr[8]);
        if (obj_conf < conf_threshold_) continue;

        softmax(ptr + 9, color_probs, 4);
        int color_id = 0;
        float max_c = color_probs[0];
        for (int c = 1; c < 4; ++c) if (color_probs[c] > max_c) { max_c = color_probs[c]; color_id = c; }

        softmax(ptr + 13, digit_probs, 9);
        int digit_id = 0;
        float max_d = digit_probs[0];
        for (int d = 1; d < 9; ++d) if (digit_probs[d] > max_d) { max_d = digit_probs[d]; digit_id = d; }

        // // 颜色过滤（根据 detect_color，假设模型输出 0=红, 1=蓝）
        // if (detect_color == 0 && color_id != 1) continue;   // 只检测蓝色
        // if (detect_color == 1 && color_id != 0) continue;   // 只检测红色

        float final_conf = obj_conf * max_d;
        if (final_conf < conf_threshold_) continue;

        Object0526 obj;
        obj.prob = final_conf;
        obj.color_id = color_id;   // 原始模型输出
        obj.digit_id = digit_id;
        for (int j = 0; j < 8; ++j) obj.landmarks[j] = std::max(0.0f, std::min(640.0f, ptr[j]));

        float min_x = obj.landmarks[0], max_x = obj.landmarks[0];
        float min_y = obj.landmarks[1], max_y = obj.landmarks[1];
        for (int j = 2; j < 8; j += 2) {
            min_x = std::min(min_x, obj.landmarks[j]);
            max_x = std::max(max_x, obj.landmarks[j]);
            min_y = std::min(min_y, obj.landmarks[j+1]);
            max_y = std::max(max_y, obj.landmarks[j+1]);
        }
        obj.rect = cv::Rect(min_x, min_y, max_x - min_x, max_y - min_y);

        cv::Point2f p0(obj.landmarks[0], obj.landmarks[1]);
        cv::Point2f p6(obj.landmarks[6], obj.landmarks[7]);
        obj.length = cv::norm(p0 - p6);
        cv::Point2f p2(obj.landmarks[2], obj.landmarks[3]);
        obj.width = cv::norm(p0 - p2);
        obj.ratio = obj.length / (obj.width + 1e-6f);

        objects.push_back(obj);
    }

    // NMS
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    boxes.reserve(objects.size());
    scores.reserve(objects.size());
    for (const auto& o : objects) {
        boxes.push_back(o.rect);
        scores.push_back(o.prob);
    }
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, scores, conf_threshold_, nms_threshold_, indices);

    std::vector<Object0526> kept;
    kept.reserve(indices.size());
    for (int idx : indices) kept.push_back(objects[idx]);
    return kept;
}

void TensorrtInferEngine0526::softmax(const float* input, float* output, int len) {
    float max_val = *std::max_element(input, input + len);
    float sum = 0.0f;
    for (int i = 0; i < len; ++i) {
        output[i] = std::exp(input[i] - max_val);
        sum += output[i];
    }
    for (int i = 0; i < len; ++i) output[i] /= sum;
}

// ---------- TensorRTYolo0526 实现 ----------
TensorRTYolo0526::TensorRTYolo0526(const std::string& config_path, bool debug)
    : debug_(debug) {
    auto yaml = YAML::LoadFile(config_path);
    auto engine_path = yaml["trt_engine_path_0526"].as<std::string>();
    detect_color_ = yaml["detect_color"].as<int>(-1);
    min_confidence_ = yaml["min_confidence"].as<double>(0.65);
    engine_ = std::make_unique<TensorrtInferEngine0526>(engine_path);
}

std::list<Armor> TensorRTYolo0526::detect(const cv::Mat& img, int frame_count) {
    if (img.empty()) {
        tools::logger()->warn("Empty image in TensorRTYolo0526::detect");
        return {};
    }


    std::vector<Object0526> objects;
    YOLOFrameData result;
    bool got = false;
    engine_->infer(img, frame_count, detect_color_, objects, result);

    return convertToArmors(objects, result.frame, frame_count);
}

YOLOFrameData TensorRTYolo0526::detect(YOLOFrameData frame_data, int frame_count) {
    std::vector<Object0526> objects;
    YOLOFrameData result;
    bool has = engine_->async_enabled_infer(frame_data.frame, frame_data,
                                            frame_count, detect_color_,
                                            objects, result);
    if (!has) return YOLOFrameData();  // is_empty = true

    result.armors = convertToArmors(objects, result.frame, frame_count);
    result.is_empty = false;
    return result;
}

std::list<Armor> TensorRTYolo0526::postprocess(double, cv::Mat&, const cv::Mat&, int) {
    return {};
}

std::list<Armor> TensorRTYolo0526::convertToArmors(const std::vector<Object0526>& objects,
                                                   const cv::Mat& img,
                                                   int /*frame_count*/) {
    std::list<Armor> armors;
    float sx = static_cast<float>(img.cols) / TensorrtInferEngine0526::IMAGE_WIDTH;
    float sy = static_cast<float>(img.rows) / TensorrtInferEngine0526::IMAGE_HEIGHT;

    for (const auto& obj : objects) {
        if (obj.prob < min_confidence_) continue;
        // 颜色过滤（与 decode 中一致）
        if (detect_color_ == 0 && obj.color_id != 1) continue;
        if (detect_color_ == 1 && obj.color_id != 0) continue;

        std::vector<cv::Point2f> pts;
        for (int i = 0; i < 8; i += 2) {
            pts.emplace_back(obj.landmarks[i] * sx, obj.landmarks[i+1] * sy);
        }
        sortKeypoints(pts);

        cv::Rect box(obj.rect.x * sx, obj.rect.y * sy,
                     obj.rect.width * sx, obj.rect.height * sy);

        // 颜色映射：模型 0=红, 1=蓝 → 构造函数期望 0=蓝, 1=红
        int color = (obj.color_id == 0) ? 0 : (obj.color_id == 1) ? 1 : 2;

        // 使用构造函数 (color_id, num_id, confidence, box, points)
        Armor armor(color, obj.digit_id, obj.prob, box, pts);

        // 过滤无效类型
        if (!checkName(armor) || !checkType(armor)) continue;

        // 计算归一化中心（Armor 构造函数中已计算 center，这里只需设置 center_norm）
        armor.center_norm = cv::Point2f(armor.center.x / img.cols, armor.center.y / img.rows);

        armors.push_back(armor);
    }

    if (debug_) drawDetections(img, armors, 0);
    return armors;
}

void TensorRTYolo0526::sortKeypoints(std::vector<cv::Point2f>& pts) {
    if (pts.size() != 4) return;
    std::sort(pts.begin(), pts.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    cv::Point2f top_left = pts[0].x < pts[1].x ? pts[0] : pts[1];
    cv::Point2f top_right = pts[0].x < pts[1].x ? pts[1] : pts[0];
    cv::Point2f bottom_left = pts[2].x < pts[3].x ? pts[2] : pts[3];
    cv::Point2f bottom_right = pts[2].x < pts[3].x ? pts[3] : pts[2];
    pts = {top_left, top_right, bottom_right, bottom_left};
}

bool TensorRTYolo0526::checkName(const Armor& armor) const {
    return armor.name != ArmorName::not_armor;
}

bool TensorRTYolo0526::checkType(const Armor& armor) const {
    if (armor.type == ArmorType::small) {
        return armor.name != ArmorName::one && armor.name != ArmorName::base;
    } else {
        return armor.name != ArmorName::two && armor.name != ArmorName::sentry &&
               armor.name != ArmorName::outpost;
    }
}

void TensorRTYolo0526::drawDetections(const cv::Mat& img, const std::list<Armor>& armors,
                                      int /*frame_count*/) const {
    if (!debug_) return;
    auto display = img.clone();
    for (const auto& armor : armors) {
        tools::draw_points(display, armor.points, {0, 255, 255});
        // cv::rectangle(display, armor.box, {0, 255, 0}, 2);
        std::string info = fmt::format("{:.2f}", armor.confidence);
        tools::draw_text(display, info, armor.center, {255, 255, 255});
    }
    cv::resize(display, display, {}, 0.5, 0.5);
    cv::imshow("TensorRT 0526 Detection", display);
}

} // namespace auto_aim
