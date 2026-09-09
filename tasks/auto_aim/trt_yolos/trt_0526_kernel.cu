#include <cuda_runtime.h>
#include <cuda_fp16.h>

__global__ void preprocess_kernel(const unsigned char* src, half* dst,
                                  int src_width, int src_height, int src_step,
                                  int dst_width, int dst_height) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_width || y >= dst_height) return;

    float sx = (float)x * src_width / dst_width;
    float sy = (float)y * src_height / dst_height;
    int x0 = (int)sx, y0 = (int)sy;
    int x1 = min(x0 + 1, src_width - 1);
    int y1 = min(y0 + 1, src_height - 1);
    float dx = sx - x0, dy = sy - y0;

    for (int c = 0; c < 3; ++c) {
        float v00 = src[y0 * src_step + x0 * 3 + c];
        float v10 = src[y0 * src_step + x1 * 3 + c];
        float v01 = src[y1 * src_step + x0 * 3 + c];
        float v11 = src[y1 * src_step + x1 * 3 + c];
        float val = (1-dx)*(1-dy)*v00 + dx*(1-dy)*v10 + (1-dx)*dy*v01 + dx*dy*v11;
        val /= 255.0f;

        int dst_c = (c == 0) ? 2 : (c == 2) ? 0 : 1;
        dst[dst_c * dst_height * dst_width + y * dst_width + x] = __float2half(val);
    }
}

extern "C" void launchPreprocess(const unsigned char* src, half* dst,
                                 int src_width, int src_height, int src_step,
                                 int dst_width, int dst_height,
                                 cudaStream_t stream) {
    dim3 block(32, 32);
    dim3 grid((dst_width + block.x - 1) / block.x, (dst_height + block.y - 1) / block.y);
    preprocess_kernel<<<grid, block, 0, stream>>>(
        src, dst, src_width, src_height, src_step, dst_width, dst_height);
}
