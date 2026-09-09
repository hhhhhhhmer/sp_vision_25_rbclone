#include "trt_yolo11_buff_kernel.h"

#include <cuda_runtime.h>

namespace
{
__device__ __forceinline__ float read_bilinear(
  const unsigned char * src, std::size_t src_step, int src_width, int src_height,
  float x, float y, int channel)
{
  // warpAffine's default inverse mapping samples the source at output / scale.
  const int x0 = static_cast<int>(floorf(x));
  const int y0 = static_cast<int>(floorf(y));
  const int x1 = x0 + 1;
  const int y1 = y0 + 1;
  const float dx = x - static_cast<float>(x0);
  const float dy = y - static_cast<float>(y0);

  // Match BORDER_CONSTANT: samples outside the source image contribute zero,
  // which preserves the partially covered edge pixels of a scaled image.
  const float p00 =
    x0 < src_width && y0 < src_height
      ? static_cast<float>(src[static_cast<std::size_t>(y0) * src_step + x0 * 3 + channel])
      : 0.0f;
  const float p10 =
    x1 < src_width && y0 < src_height
      ? static_cast<float>(src[static_cast<std::size_t>(y0) * src_step + x1 * 3 + channel])
      : 0.0f;
  const float p01 =
    x0 < src_width && y1 < src_height
      ? static_cast<float>(src[static_cast<std::size_t>(y1) * src_step + x0 * 3 + channel])
      : 0.0f;
  const float p11 =
    x1 < src_width && y1 < src_height
      ? static_cast<float>(src[static_cast<std::size_t>(y1) * src_step + x1 * 3 + channel])
      : 0.0f;
  const float top = fmaf(dx, p10 - p00, p00);
  const float bottom = fmaf(dx, p11 - p01, p01);
  return fmaf(dy, bottom - top, top) * (1.0f / 255.0f);
}

__global__ void yolo11_buff_preprocess_kernel(
  const unsigned char * __restrict__ src, int src_width, int src_height, std::size_t src_step,
  float * __restrict__ dst, int dst_width, int dst_height, float scale)
{
  const int x = blockIdx.x * blockDim.x + threadIdx.x;
  const int y = blockIdx.y * blockDim.y + threadIdx.y;
  if (x >= dst_width || y >= dst_height) return;

  const std::size_t plane = static_cast<std::size_t>(dst_width) * dst_height;
  const std::size_t offset = static_cast<std::size_t>(y) * dst_width + x;
  const float src_x = static_cast<float>(x) / scale;
  const float src_y = static_cast<float>(y) / scale;
  // Input cameras provide packed BGR, while the network consumes RGB.
  const float b = read_bilinear(src, src_step, src_width, src_height, src_x, src_y, 0);
  const float g = read_bilinear(src, src_step, src_width, src_height, src_x, src_y, 1);
  const float r = read_bilinear(src, src_step, src_width, src_height, src_x, src_y, 2);
  dst[offset] = r;
  dst[plane + offset] = g;
  dst[2 * plane + offset] = b;
}
}  // namespace

extern "C" void launch_yolo11_buff_preprocess(
  const unsigned char * src, int src_width, int src_height, std::size_t src_step,
  float * dst, int dst_width, int dst_height, float scale, cudaStream_t stream)
{
  const dim3 block(32, 8);
  const dim3 grid(
    (static_cast<unsigned>(dst_width) + block.x - 1) / block.x,
    (static_cast<unsigned>(dst_height) + block.y - 1) / block.y);
  yolo11_buff_preprocess_kernel<<<grid, block, 0, stream>>>(
    src, src_width, src_height, src_step, dst, dst_width, dst_height, scale);
}
