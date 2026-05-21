#include "preprocess.hpp"
#include <cuda_runtime.h>

__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src,
    int src_w, int src_h, int src_step,
    float* __restrict__ dst,
    int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y,
    int new_w, int new_h,
    float mean0, float mean1, float mean2,
    float std0, float std1, float std2,
    bool swapRB)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) return;

    float b, g, r;

    if (x < new_w && y < new_h) {
        float sx = x * scale_inv_x;
        float sy = y * scale_inv_y;

        int x0 = static_cast<int>(floorf(sx));
        int y0 = static_cast<int>(floorf(sy));
        int x1 = min(x0 + 1, src_w - 1);
        int y1 = min(y0 + 1, src_h - 1);

        float dx = sx - x0;
        float dy = sy - y0;

        int row0 = y0 * src_step;
        int row1 = y1 * src_step;
        int x0_3 = x0 * 3;
        int x1_3 = x1 * 3;

        float w00 = (1.0f - dx) * (1.0f - dy);
        float w10 = dx * (1.0f - dy);
        float w01 = (1.0f - dx) * dy;
        float w11 = dx * dy;

        b = src[row0 + x0_3 + 0] * w00 + src[row0 + x1_3 + 0] * w10
          + src[row1 + x0_3 + 0] * w01 + src[row1 + x1_3 + 0] * w11;
        g = src[row0 + x0_3 + 1] * w00 + src[row0 + x1_3 + 1] * w10
          + src[row1 + x0_3 + 1] * w01 + src[row1 + x1_3 + 1] * w11;
        r = src[row0 + x0_3 + 2] * w00 + src[row0 + x1_3 + 2] * w10
          + src[row1 + x0_3 + 2] * w01 + src[row1 + x1_3 + 2] * w11;
    } else {
        b = g = r = 114.0f;
    }

    float mean[3] = {mean0, mean1, mean2};
    float stdv[3] = {std0, std1, std2};

    int pixel_idx = y * dst_w + x;
    float vals[3] = {b, g, r};

    for (int c = 0; c < 3; ++c) {
        int src_c = swapRB ? (2 - c) : c;
        float v = vals[src_c] * (1.0f / 255.0f);
        v = (v - mean[c]) / stdv[c];
        dst[c * dst_w * dst_h + pixel_idx] = v;
    }
}

void launch_preprocess(
    const uint8_t* src, int src_w, int src_h, int src_step,
    float* dst, int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y, int new_w, int new_h,
    const float mean[3], const float std[3],
    bool swapRB,
    cudaStream_t stream)
{
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);

    preprocess_kernel<<<grid, block, 0, stream>>>(
        src, src_w, src_h, src_step,
        dst, dst_w, dst_h,
        scale_inv_x, scale_inv_y, new_w, new_h,
        mean[0], mean[1], mean[2],
        std[0], std[1], std[2],
        swapRB);
}
