#include <cuda_runtime.h>

__global__ void resize_bgr_to_rgb_chw_kernel(const unsigned char* src, int src_w, int src_h,
                                             float* dst, int dst_w, int dst_h,
                                             float scale_x, float scale_y) {
    int dst_x = blockIdx.x * blockDim.x + threadIdx.x;
    int dst_y = blockIdx.y * blockDim.y + threadIdx.y;
    if (dst_x >= dst_w || dst_y >= dst_h) return;

    int src_x = min(int(dst_x * scale_x), src_w - 1);
    int src_y = min(int(dst_y * scale_y), src_h - 1);
    int src_idx = (src_y * src_w + src_x) * 3;  // BGR

    float r = src[src_idx + 2] / 255.0f;
    float g = src[src_idx + 1] / 255.0f;
    float b = src[src_idx + 0] / 255.0f;

    int dst_idx = dst_y * dst_w + dst_x;
    dst[0 * dst_h * dst_w + dst_idx] = r;
    dst[1 * dst_h * dst_w + dst_idx] = g;
    dst[2 * dst_h * dst_w + dst_idx] = b;
}

// 封装为 C 接口函数，供 .cpp 调用
extern "C" void launch_preprocess(const unsigned char* src, int src_w, int src_h,
                                  float* dst, int dst_w, int dst_h,
                                  cudaStream_t stream) {
    float scale_x = (float)src_w / dst_w;
    float scale_y = (float)src_h / dst_h;
    dim3 block(32, 32);
    dim3 grid((dst_w + block.x - 1) / block.x, (dst_h + block.y - 1) / block.y);
    resize_bgr_to_rgb_chw_kernel<<<grid, block, 0, stream>>>(src, src_w, src_h, dst, dst_w, dst_h, scale_x, scale_y);
}