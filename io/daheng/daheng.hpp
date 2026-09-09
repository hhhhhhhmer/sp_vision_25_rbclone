#ifndef DAHENG_CAMERA_HPP
#define DAHENG_CAMERA_HPP

#include <thread>
#include <atomic>
#include <memory>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "GxIAPI.h"  // 大恒相机SDK头文件
#include "DxImageProc.h"

#include "tools/thread_safe_queue.hpp"
#include "io/camera.hpp"
#include "tools/logger.hpp"

namespace io
{


class DahengCamera  : public CameraBase{
public:

    static void initSDK(){
        // 初始化SDK
        GX_STATUS status = GXInitLib();
        if (status != GX_STATUS_SUCCESS) {
            tools::logger()->error("[Daheng] 大恒相机初始化失败，错误码: {:#x}", status);
            return;
        }
    }
    /**
     * @brief 构造函数
     * @param exposure_us 曝光时间(微秒)
     * @param gain 增益值
     * @param frame_rate 帧率
     * @param serial_number 相机序列号(为空时使用第一个相机)
     */
    DahengCamera(std::string camera_sn, 
                                double exposure_us, 
                                double gain, 
                                double gamma,
                                bool flip,
                                bool mirror
                            );
    
    ~DahengCamera();

    bool capture_stop();
    
    /**
     * @brief 读取图像
     * @param img 输出的图像
     * @param timestamp 时间戳
     */
    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) override;
    bool try_read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
    void clear_camera_frame_buffer() override { queue_.clear(); }
private:

    struct CameraData {
        cv::Mat img;
        std::chrono::steady_clock::time_point timestamp;
    };
    // 相机操作
    bool enum_and_check_camera();  // 枚举并检查相机
    bool initialize_camera();
    bool open_camera();
    bool close_camera();

    cv::Mat getFrame( );
    void ProcessData(void *pImageBuf, void *pImageRaw8Buf, void *pImageRGBBuf, int nImageWidth, int nImageHeight,
                        int nPixelFormat, int nPixelColorFilter, bool flip , bool mirror ) ;
private:

    // 相机参数
    // std::string camera_sn_;
    double exposure_us_;
    double gain_;
    double gamma_;
    double frame_rate_;
    std::string open_content_; 
    bool flip_ = false;// 垂直翻转
    bool mirror_ = false;// 水平镜像
    
    // 相机句柄和状态
    GX_DEV_HANDLE hDevice = nullptr;
    GX_OPEN_PARAM* open_param_ = nullptr;
    int64_t PixelFormat = GX_PIXEL_FORMAT_BAYER_GR8;
    int64_t ColorFilter = GX_COLOR_FILTER_NONE;
    GX_FRAME_DATA frameData{};
    void *pRaw8Buffer = nullptr;
    void *pMirrorBuffer = nullptr;
    void *pRGBframeData = nullptr;
    void *pGammaLut = nullptr;
    
    // std::atomic<bool> connected_{false};
    // std::atomic<bool> capturing_{false};
    
    // 线程控制
    std::atomic<bool> daemon_quit_{false};
    std::atomic<bool> capture_quit_{false};
    // std::atomic<bool> is_stop_collecting{false};// 断采集
    std::thread daemon_thread_;
    std::thread capture_thread_;
    size_t stop_collecting_num = 0;
    
    // 数据队列
    tools::ThreadSafeQueue<CameraData, true> queue_;
    
    // SDK状态
    bool sdk_initialized_ ;//= false;
    
    // 图像参数缓存
    size_t image_width_ = 0;
    size_t image_height_ = 0;
    // GX_PIXEL_FORMAT_ENTRY pixel_format_ = GX_PIXEL_FORMAT_MONO8;
    
    // 配置参数
    bool trigger_mode_ = false;
    bool auto_white_balance_ = true;
    

    void pause() override;
    void resume() override;

    std::mutex pause_mutex_;
    std::condition_variable pause_cv_;
};

}  // namespace io

#endif  // DAHENG_CAMERA_HPP
