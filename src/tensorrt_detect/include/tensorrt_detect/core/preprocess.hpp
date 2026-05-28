#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include <cuda_runtime.h>
#include <cstdint>

/**
 * @brief Launch CUDA kernel for image preprocessing (raw pointer version).
 *
 * Performs resize (bilinear) + letterbox (top-left, pad=114) +
 * BGR->RGB swap + normalization (1/255) + optional ImageNet standardization,
 * all on GPU. Output is NCHW float directly into the model input buffer.
 */
void launch_preprocess(
    const uint8_t* src, int src_w, int src_h, int src_step,
    float* dst, int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y, int new_w, int new_h,
    const float mean[3], const float std[3],
    bool swapRB,
    cudaStream_t stream);

/**
 * @brief Launch CUDA kernel for image preprocessing (texture memory version).
 *
 * Uses a CUDA texture object bound to the source image for hardware-accelerated
 * bilinear interpolation. Parameters are identical to launch_preprocess()
 * except the source image is accessed via the texture object.
 */
void launch_preprocess_tex(
    cudaTextureObject_t tex,
    int src_w, int src_h,
    float* dst, int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y, int new_w, int new_h,
    const float mean[3], const float std[3],
    bool swapRB,
    cudaStream_t stream);

#endif
