#ifndef AUTO_BUFF_TRT_YOLO11_BUFF_KERNEL_H
#define AUTO_BUFF_TRT_YOLO11_BUFF_KERNEL_H

#include <cuda_runtime_api.h>

#include <cstddef>

#ifdef __cplusplus
extern "C" {
#endif

// Fuses top-left letterbox, bilinear sampling, BGR->RGB conversion and /255.
// The source is packed BGR with src_step bytes per row; dst is float NCHW RGB.
void launch_yolo11_buff_preprocess(
  const unsigned char * src, int src_width, int src_height, std::size_t src_step,
  float * dst, int dst_width, int dst_height, float scale, cudaStream_t stream);

#ifdef __cplusplus
}
#endif

#endif
