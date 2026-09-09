#ifndef PREPROCESS_KERNEL_H
#define PREPROCESS_KERNEL_H

#include <cuda_runtime.h>

#ifdef __cplusplus
extern "C" {
#endif

void launch_preprocess(const unsigned char* src, int src_w, int src_h,
                       float* dst, int dst_w, int dst_h,
                       cudaStream_t stream);

#ifdef __cplusplus
}
#endif

#endif