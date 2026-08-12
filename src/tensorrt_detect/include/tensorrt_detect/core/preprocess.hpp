/**
 * @file preprocess.hpp
 * @brief TensorRT 输入张量的 CUDA 预处理 kernel 启动接口。
 *
 * CPU BGR 图像先由 Model 拷入 pinned host staging，再异步 H2D；本接口在同一 CUDA
 * Stream 上完成 resize/letterbox、通道转换、归一化并直接写 NCHW float binding。
 */
#ifndef __PREPROCESS_HPP__
#define __PREPROCESS_HPP__

#include <cuda_runtime.h>
#include <cstdint>

/**
 * @brief 启动基于原始 device pointer 的图像预处理 kernel。
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
 * @brief 启动使用 CUDA texture 双线性采样的预处理 kernel。
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
