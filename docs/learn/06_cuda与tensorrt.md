# 06 CUDA 与 TensorRT

> 本项目使用 **NVIDIA CUDA** 进行 GPU 并行图像预处理，使用 **TensorRT 10** 进行深度学习模型的高性能推理。两者结合使得单帧处理延迟可控制在毫秒级别。本章从零讲解 CUDA 编程基础和 TensorRT 10 API 的使用。

---

## 6.1 CUDA 基础

### 6.1.1 CUDA 编程模型

CUDA 程序由**主机端（Host）**和**设备端（Device）**代码组成：
- **Host**：CPU 上执行的 C++ 代码，负责内存分配、Kernel 启动、数据拷贝
- **Device**：GPU 上执行的并行代码，以 **Kernel**（`__global__` 函数）形式编写

```cpp
// Host 代码
void launch_preprocess(...) {
    dim3 block(16, 16);
    dim3 grid((dst_w + 15) / 16, (dst_h + 15) / 16);
    preprocess_kernel<<<grid, block, 0, stream>>>(...);  // 启动 Kernel
}

// Device 代码（Kernel）
__global__ void preprocess_kernel(...) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) return;
    // ... 每个线程处理一个像素
}
```

### 6.1.2 线程层次结构

```
Grid（网格）
├── Block(0,0)          Block(1,0)
│   ├── Thread(0,0)       ├── Thread(0,0)
│   ├── Thread(1,0)       ├── Thread(1,0)
│   └── ...               └── ...
└── Block(0,1)
    ├── Thread(0,0)
    └── ...
```

- **Grid**：由多个 Block 组成，三维索引 `(blockIdx.x, blockIdx.y, blockIdx.z)`
- **Block**：由多个 Thread 组成，三维索引 `(threadIdx.x, threadIdx.y, threadIdx.z)`
- **Warp**：32 个线程为一组，是 GPU 调度的最小单位

本项目中的图像预处理使用 2D Grid / 2D Block：

```cpp
dim3 block(16, 16);  // 每个 Block 256 线程
dim3 grid((dst_w + block.x - 1) / block.x,
          (dst_h + block.y - 1) / block.y);
```

### 6.1.3 CUDA Stream

Stream 是 GPU 上的任务队列，同一 Stream 中的操作按顺序执行，不同 Stream 可并发。

```cpp
cudaStream_t stream = nullptr;  // 默认流（NULL stream）

// TensorRT 上下文绑定 Stream
context->enqueueV3(stream);     // 异步推理

cudaStreamSynchronize(stream);  // 同步等待 Stream 完成
```

**异步执行的关键**：
1. `cudaMemcpyAsync`（异步内存拷贝）
2. `kernel<<<..., stream>>>`（异步 Kernel 启动）
3. `context->enqueueV3(stream)`（异步推理）
4. 最后 `cudaStreamSynchronize(stream)` 等待全部完成

---

## 6.2 图像预处理 CUDA Kernel

本项目的 `preprocess.cu` 实现了完整的 GPU 图像预处理流水线：

### 6.2.1 功能清单

| 步骤 | 操作 | 执行位置 |
|:---|:---|:---|
| 1 | Bilinear Resize（双线性插值缩放） | GPU |
| 2 | Letterbox Padding（灰边填充） | GPU |
| 3 | BGR → RGB 通道交换 | GPU |
| 4 | Normalize（除以 255） | GPU |
| 5 | Standardization（ImageNet 均值方差） | GPU（可选） |
| 6 | HWC → CHW 重排 | GPU |

### 6.2.2 Kernel 源码解析

```cuda
#include <cuda_runtime.h>

__global__ void preprocess_kernel(
    const uint8_t* __restrict__ src,   // 输入图像（BGR, HWC）
    int src_w, int src_h, int src_step,
    float* __restrict__ dst,           // 输出（RGB, CHW）
    int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y,  // 缩放比例倒数
    int new_w, int new_h,              // 缩放后尺寸（不含 padding）
    float mean0, float mean1, float mean2,
    float std0, float std1, float std2,
    bool swapRB)                       // 是否交换 R/B
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) return;

    float b, g, r;

    // ===== 步骤 1 & 2: Resize + Letterbox =====
    if (x < new_w && y < new_h) {
        // 双线性插值
        float sx = x * scale_inv_x;
        float sy = y * scale_inv_y;

        int x0 = static_cast<int>(floorf(sx));
        int y0 = static_cast<int>(floorf(sy));
        int x1 = min(x0 + 1, src_w - 1);
        int y1 = min(y0 + 1, src_h - 1);

        float dx = sx - x0;
        float dy = sy - y0;

        // 四个邻域像素的权重
        float w00 = (1.0f - dx) * (1.0f - dy);
        float w10 = dx * (1.0f - dy);
        float w01 = (1.0f - dx) * dy;
        float w11 = dx * dy;

        // 读取并插值
        int row0 = y0 * src_step;
        int row1 = y1 * src_step;
        b = src[row0 + x0*3 + 0] * w00 + src[row0 + x1*3 + 0] * w10
          + src[row1 + x0*3 + 0] * w01 + src[row1 + x1*3 + 0] * w11;
        g = src[row0 + x0*3 + 1] * w00 + src[row0 + x1*3 + 1] * w10
          + src[row1 + x0*3 + 1] * w01 + src[row1 + x1*3 + 1] * w11;
        r = src[row0 + x0*3 + 2] * w00 + src[row0 + x1*3 + 2] * w10
          + src[row1 + x0*3 + 2] * w01 + src[row1 + x1*3 + 2] * w11;
    } else {
        // Letterbox padding: 灰色填充 (114, 114, 114)
        b = g = r = 114.0f;
    }

    // ===== 步骤 3, 4, 5, 6: 通道交换 + 归一化 + 标准化 + CHW =====
    int pixel_idx = y * dst_w + x;
    float vals[3] = {b, g, r};

    for (int c = 0; c < 3; ++c) {
        int src_c = swapRB ? (2 - c) : c;   // BGR->RGB 交换
        float v = vals[src_c] * (1.0f / 255.0f);  // 归一化到 [0,1]
        v = (v - mean[c]) / stdv[c];        // 标准化
        dst[c * dst_w * dst_h + pixel_idx] = v;   // CHW 排列
    }
}
```

### 6.2.3 Host 端调用封装

```cpp
// preprocess.hpp
void launch_preprocess(
    const uint8_t* src, int src_w, int src_h, int src_step,
    float* dst, int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y, int new_w, int new_h,
    const float mean[3], const float std[3],
    bool swapRB,
    cudaStream_t stream);
```

### 6.2.4 内存管理策略

```cpp
class Model {
private:
    void* buffers[2] = {nullptr, nullptr};  // TensorRT 绑定缓冲区
    
    // Pinned Host Memory：允许异步 DMA 传输
    float* prob_ = nullptr;      // 输入缓冲区
    float* hOutput_ = nullptr;   // 输出缓冲区
    
    // GPU 内存
    void* gpuInputBuffer8U_ = nullptr;   // 原始图像 GPU 内存
    void* batchInputBuffer_ = nullptr;   // Batch 输入 GPU 内存
    void* batchOutputBuffer_ = nullptr;  // Batch 输出 GPU 内存
};
```

**Pinned Memory（页锁定内存）**：
- 普通主机内存：`cudaMemcpy` 需要 CPU 参与，同步阻塞
- Pinned 内存：`cudaMemcpyAsync` 可由 DMA 控制器直接传输，完全异步

---

## 6.3 TensorRT 10 推理引擎

### 6.3.1 TensorRT 核心概念

| 类 | 作用 |
|:---|:---|
| `nvinfer1::IRuntime` | 运行时环境，反序列化引擎 |
| `nvinfer1::ICudaEngine` | 优化后的推理引擎（不可变） |
| `nvinfer1::IExecutionContext` | 执行上下文（可创建多个，支持并发） |
| `nvinfer1::IHostMemory` | 序列化引擎数据的内存缓冲区 |

### 6.3.2 引擎加载流程

```cpp
Model::Model(const std::string modelPath, ...) {
    // 1. 创建 Runtime
    runtime = nvinfer1::createInferRuntime(gLogger);
    
    // 2. 读取 .engine 文件
    std::ifstream file(modelPath, std::ios::binary);
    file.seekg(0, std::ios::end);
    size_t size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> engineData(size);
    file.read(engineData.data(), size);
    
    // 3. 反序列化引擎
    engine = runtime->deserializeCudaEngine(engineData.data(), size);
    
    // 4. 创建执行上下文
    context = engine->createExecutionContext();
    
    // 5. 获取绑定信息
    inputName_ = engine->getIOTensorName(0);
    outputName_ = engine->getIOTensorName(1);
    
    // 6. 分配 CUDA 内存
    cudaMalloc(&buffers[0], inputSize);
    cudaMalloc(&buffers[1], outputSize);
    
    // 7. 创建 CUDA Stream
    cudaStreamCreate(&stream);
    
    // 8. 分配 Pinned Host Memory
    cudaMallocHost(&prob_, probSize);
    cudaMallocHost(&hOutput_, hOutputSize);
}
```

### 6.3.3 TensorRT 10 API 变化

TensorRT 10 相比 8.x 有重要 API 变更，本项目使用的是 **TRT 10 API**：

| TRT 8.x | TRT 10 | 说明 |
|:---|:---|:---|
| `context->enqueueV2(...)` | `context->enqueueV3(stream)` | 新执行 API |
| `engine->getBindingIndex(...)` | `engine->getIOTensorName(...)` | IO Tensor 名称绑定 |
| `context->setBindingBuffers(...)` | `context->setTensorAddress(...)` | 设置 Tensor 地址 |
| `engine->getBindingDataType(...)` | `engine->getTensorDataType(...)` | 数据类型查询 |

### 6.3.4 推理执行

```cpp
bool Model::Detect(const cv::Mat &frame) {
    // 1. 图像预处理（GPU）
    preprocessing(frame);
    
    // 2. 设置输入 Tensor 地址
    context->setTensorAddress(inputName_.c_str(), buffers[0]);
    context->setTensorAddress(outputName_.c_str(), buffers[1]);
    
    // 3. 异步执行推理
    bool status = context->enqueueV3(stream);
    if (!status) return false;
    
    // 4. 同步等待
    cudaStreamSynchronize(stream);
    
    // 5. 后处理（CPU）
    postprocessing();
    
    return true;
}
```

### 6.3.5 Batch 推理

本项目支持 Batch 分类推理，用于同时识别多个装甲板：

```cpp
std::vector<int> Model::predictClassBatch(const std::vector<cv::Mat>& rois) {
    int batchSize = static_cast<int>(rois.size());
    
    // 1. 确保 Batch 缓冲区足够
    ensureBatchBuffers(batchInputBytes, batchOutputBytes);
    
    // 2. 批量预处理
    for (int i = 0; i < batchSize; ++i) {
        // 每个 ROI 预处理到 batchInputBuffer_ 的对应位置
    }
    
    // 3. 设置动态 Batch 维度
    nvinfer1::Dims4 inputDims{batchSize, 3, input_h, input_w};
    context->setInputShape(inputName_.c_str(), inputDims);
    
    // 4. 执行 Batch 推理
    context->enqueueV3(stream);
    cudaStreamSynchronize(stream);
    
    // 5. Batch 后处理
    return postprocessBatch(...);
}
```

---

## 6.4 CUDA 并发安全

### 6.4.1 全局 CUDA 互斥锁

由于本项目使用单线程容器，但为了防御未来可能的多线程扩展，实现了全局 CUDA 锁：

```cpp
// model.hpp
namespace cuda_guard {
    inline std::mutex& getCudaMutex() {
        static std::mutex mtx;
        return mtx;
    }
}
```

### 6.4.2 CUDA Primary Context 预初始化

```cpp
// detect_node.cpp 和 pose_node.cpp 构造函数中
cudaFree(0);  // 触发 CUDA Primary Context 初始化
```

**原因**：如果多个节点同时首次调用 CUDA API，会竞争初始化 Primary Context，导致 SIGSEGV。在构造函数中提前初始化可消除竞争。

---

## 6.5 CMake 中的 CUDA 配置

```cmake
# 声明 CUDA 语言
project(tensorrt_detect LANGUAGES CXX CUDA)

# C++ 和 CUDA 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)

# CUDA 架构（本项目使用 SM 86：Ampere，如 RTX 3060/3070/3080）
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 86)
endif()

# 查找 CUDA Toolkit
find_package(CUDAToolkit REQUIRED)

# 链接 CUDA 运行时
target_link_libraries(tensorrt_detect_core
    nvinfer
    nvinfer_plugin
    cudart
)
```

**SM 架构对照表**：

| 架构 | SM 版本 | 代表显卡 |
|:---|:---|:---|
| Turing | 75 | RTX 2060/2070/2080 |
| Ampere | 80/86 | RTX 3060/3070/3080/3090, A100 |
| Ada Lovelace | 89/90 | RTX 4090 |

---

## 6.6 本章小结

| 技术点 | 作用 | 本项目文件 |
|:---|:---|:---|
| CUDA Kernel | GPU 并行图像预处理 | `preprocess.cu` |
| CUDA Stream | 异步执行流水线 | `model.cpp` |
| Pinned Memory | 异步 DMA 传输 | `model.hpp` |
| TensorRT Runtime | 反序列化 .engine | `model.cpp` |
| TensorRT Engine | 优化后的推理图 | `model.cpp` |
| TensorRT Context | 推理执行环境 | `model.cpp` |
| enqueueV3 | TRT 10 异步推理 API | `model.cpp` |
| Batch 推理 | 多目标同时分类 | `model.cpp` |
| CUDA 互斥锁 | 防止并发 CUDA 冲突 | `model.hpp` |
