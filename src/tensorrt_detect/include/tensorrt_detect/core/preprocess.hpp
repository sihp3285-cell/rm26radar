#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include <cuda_runtime.h>
#include <cstdint>

/**
 * @brief Launch CUDA kernel for image preprocessing.
 *
 * Performs resize (bilinear) + letterbox (top-left, pad=114) +
 * BGR->RGB swap + normalization (1/255) + optional ImageNet standardization,
 * all on GPU. Output is NCHW float directly into the model input buffer.
 *
 * @param src       Source image data (BGR, uint8, HWC) on device
 * @param src_w     Source width
 * @param src_h     Source height
 * @param src_step  Source row stride in bytes
 * @param dst       Destination float buffer (CHW) on device
 * @param dst_w     Destination width (model input width)
 * @param dst_h     Destination height (model input height)
 * @param scale_inv_x  src_w / new_w
 * @param scale_inv_y  src_h / new_h
 * @param new_w     Width after resize (before padding)
 * @param new_h     Height after resize (before padding)
 * @param mean      Per-channel mean for standardization
 * @param std       Per-channel std for standardization
 * @param swapRB    If true, swap R and B channels (BGR->RGB)
 * @param stream    CUDA stream
 */
void launch_preprocess(
    const uint8_t* src, int src_w, int src_h, int src_step,
    float* dst, int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y, int new_w, int new_h,
    const float mean[3], const float std[3],
    bool swapRB,
    cudaStream_t stream);

#endif
