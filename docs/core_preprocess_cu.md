# `preprocess.cu` + `preprocess.hpp` 逐行讲解

这两份文件实现了 **CUDA GPU 图像预处理内核**，把原本在 CPU 上执行的图像预处理流水线搬到 GPU 上，是本项目中**推理预处理的关键性能优化**。

在系统中的位置：

```text
detect_node
   └── DetectPipeline
         └── Model（模型推理）
               ├── preprocessing（预处理）  ← 这里
               │     ├── CPU 路径：cv::resize + cv::dnn::blobFromImage
               │     └── GPU 路径：launch_preprocess (CUDA kernel) ← 当前使用
               ├── executeV2（TensorRT 推理）
               └── postprocessing（后处理）
```

---

# 一、设计思想：为什么需要 GPU 预处理？

## 1.1 CPU 预处理的瓶颈

旧版 `Model::preprocessing` 在 CPU 上执行以下操作：

1. `cv::resize` — 等比例缩放图像到模型输入尺寸
2. `canvas.setTo(114)` — 创建灰色填充画布
3. `resized_img.copyTo(canvas)` — 贴图到画布左上角
4. `cv::dnn::blobFromImage` — 归一化 + BGR→RGB + HWC→CHW
5. `cudaMemcpyAsync` — CPU→GPU 数据传输

这些操作**全部在 CPU 上完成**，然后才把结果拷贝到 GPU。对于 1920×1080 的输入图像，CPU 预处理约需 **3~5ms**。

## 1.2 GPU 预处理的优势

CUDA 内核把上述所有操作**在 GPU 上一步完成**：

* **消除 CPU→GPU 数据传输**：原始图像已经通过 `cudaMemcpyAsync` 传到 GPU，预处理直接在 GPU 显存上操作
* **并行化**：每个像素的处理独立，GPU 数千个 CUDA 核心同时工作
* **减少 CPU 负担**：CPU 可以同时准备下一帧数据

预计预处理时间从 3~5ms 降到 **< 0.5ms**。

---

# 第二部分：`preprocess.hpp`

```cpp
void launch_preprocess(
    const uint8_t* src, int src_w, int src_h, int src_step,
    float* dst, int dst_w, int dst_h,
    float scale_inv_x, float scale_inv_y, int new_w, int new_h,
    const float mean[3], const float std[3],
    bool swapRB,
    cudaStream_t stream);
```

---

### 参数说明

| 参数 | 含义 |
|------|------|
| `src` | 源图像数据（BGR, uint8, HWC 格式），**设备端指针** |
| `src_w / src_h` | 源图像宽高 |
| `src_step` | 源图像行步长（字节），可能含 padding |
| `dst` | 目标 float 缓冲区（CHW 格式），**设备端指针** |
| `dst_w / dst_h` | 目标尺寸（模型输入尺寸，如 640×640） |
| `scale_inv_x / scale_inv_y` | 缩放系数（`src_w / new_w`，`src_h / new_h`） |
| `new_w / new_h` | 缩放后的图像尺寸（不含 padding） |
| `mean / std` | 归一化均值和标准差（每个通道） |
| `swapRB` | 是否交换 R 和 B 通道（BGR→RGB） |
| `stream` | CUDA 流 |

---

### 调用时机

在 `Model::preprocessing` 中被调用，替代了原来的 CPU 路径：

```cpp
// 旧版 CPU 路径
cv::resize(...) → canvas.setTo(114) → copyTo → blobFromImage → cudaMemcpyAsync

// 新版 GPU 路径
cudaMemcpyAsync(src到GPU) → launch_preprocess(直接写入模型输入buffer)
```

---

# 第三部分：`preprocess.cu` 内核实现

## 一、CUDA 内核函数

```cuda
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
```

---

### `__global__`

CUDA 关键字，表示这是一个 GPU 内核函数，可以从 CPU 端调用（`<<<grid, block>>>` 语法），在 GPU 上执行。

---

### `__restrict__`

C/C++ 关键字，提示编译器该指针不会与其他指针别名（aliasing）。这让编译器做更激进的优化（如指令重排、寄存器分配）。

---

### 线程与像素的映射

```cuda
int x = blockIdx.x * blockDim.x + threadIdx.x;
int y = blockIdx.y * blockDim.y + threadIdx.y;
if (x >= dst_w || y >= dst_h) return;
```

每个 CUDA 线程处理**一个目标像素**。`x` 是列索引，`y` 是行索引。

---

## 二、Letterbox 缩放（双线性插值）

```cuda
if (x < new_w && y < new_h) {
    float sx = x * scale_inv_x;
    float sy = y * scale_inv_y;

    int x0 = static_cast<int>(floorf(sx));
    int y0 = static_cast<int>(floorf(sy));
    int x1 = min(x0 + 1, src_w - 1);
    int y1 = min(y0 + 1, src_h - 1);

    float dx = sx - x0;
    float dy = sy - y0;

    // 双线性插值
    float w00 = (1.0f - dx) * (1.0f - dy);
    float w10 = dx * (1.0f - dy);
    float w01 = (1.0f - dx) * dy;
    float w11 = dx * dy;

    b = src[row0 + x0_3 + 0] * w00 + src[row0 + x1_3 + 0] * w10
      + src[row1 + x0_3 + 0] * w01 + src[row1 + x1_3 + 0] * w11;
    g = src[row0 + x0_3 + 1] * w00 + ...
    r = src[row0 + x0_3 + 2] * w00 + ...
```

---

### 双线性插值原理

目标像素 `(x, y)` 映射回源图像的浮点坐标 `(sx, sy)`，然后取周围 4 个最近邻像素，按距离加权平均：

```text
目标像素值 = w00 × P(x0,y0) + w10 × P(x1,y0) + w01 × P(x0,y1) + w11 × P(x1,y1)
```

其中 `w00 = (1-dx)×(1-dy)`，`dx` 和 `dy` 是 `(sx, sy)` 到 `(x0, y0)` 的亚像素偏移。

---

### 为什么在 GPU 上用双线性而不是 `cv::resize`？

`cv::resize` 在 CPU 上执行，使用的是 OpenCV 的高度优化实现。但在我们的流水线中，图像已经在 GPU 上了，再拷回 CPU 做 resize 再拷回来是浪费。GPU 内核的双线性插值虽然不如 OpenCV 的 SIMD 优化极致，但**省去了两次 CPU↔GPU 拷贝**，总体更快。

---

## 三、灰色填充（Letterbox Padding）

```cuda
} else {
    b = g = r = 114.0f;
}
```

当 `x >= new_w` 或 `y >= new_h` 时，像素位于填充区域，直接填灰色（114, 114, 114）。

这与 YOLO 训练时的默认 Letterbox padding 值一致。

---

## 四、归一化 + 通道交换 + CHW 输出

```cuda
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
```

---

### 操作步骤

1. **通道交换**：如果 `swapRB=true`，把 BGR 转成 RGB（`src_c = 2-c`）
2. **归一化到 [0,1]**：`v = pixel / 255.0`
3. **标准化**：`v = (v - mean) / std`（ImageNet 预训练模型的标准预处理）
4. **CHW 输出**：写入 `dst[c * H * W + y * W + x]`

---

### CHW 内存布局

```text
dst 缓冲区：
[R0,0  R0,1  ... R0,W-1  R1,0  ... RH-1,W-1    // R 通道
 G0,0  G0,1  ... G0,W-1  G1,0  ... GH-1,W-1    // G 通道
 B0,0  B0,1  ... B0,W-1  B1,0  ... BH-1,W-1]   // B 通道
```

这是深度学习框架（TensorRT、PyTorch、ONNX）的标准输入格式。

---

## 五、启动函数

```cuda
void launch_preprocess(...) {
    dim3 block(16, 16);
    dim3 grid((dst_w + block.x - 1) / block.x,
              (dst_h + block.y - 1) / block.y);

    preprocess_kernel<<<grid, block, 0, stream>>>(...);
}
```

---

### 线程块配置

* **block = 16×16 = 256 线程/块**：这是 CUDA 的经典配置，充分利用 GPU 的 warp 调度（32 线程/warp）
* **grid = (dst_w/16) × (dst_h/16)**：覆盖整个目标图像

对于 640×640 的模型输入：

```text
grid = (40, 40) = 1600 个线程块
每个块 256 个线程
总计 409,600 个线程 = 409,600 个像素
```

GPU 可以在极短时间内并行处理所有像素。

---

### CUDA Stream

```cuda
<<<grid, block, 0, stream>>>
```

第 3 个参数 `0` 是共享内存大小（不使用共享内存）。第 4 个参数 `stream` 是 CUDA 流，确保内核与之前的 `cudaMemcpyAsync` 和之后的 `executeV2` 在同一流中顺序执行。

---

# 第四部分：与 CPU 路径的对比

| 步骤 | CPU 路径 | GPU 路径 |
|------|---------|---------|
| 图像传输 | CPU→GPU（整帧 6MB） | CPU→GPU（整帧 6MB，同） |
| 缩放 | `cv::resize`（CPU ~1ms） | CUDA kernel（GPU ~0.1ms） |
| 填充 | `canvas.setTo(114)`（CPU ~0.5ms） | 内核内条件判断（零开销） |
| 归一化 | `blobFromImage`（CPU ~1ms） | 内核内计算（零开销） |
| HWC→CHW | `blobFromImage`（CPU ~0.5ms） | 内核内输出（零开销） |
| **总预处理** | **~3ms（CPU）** | **~0.1ms（GPU）** |

GPU 路径的优势不仅是速度，更重要的是**消除了 CPU 和 GPU 之间的数据传输瓶颈**：

```text
CPU 路径：GPU ← 拷贝 ← CPU(预处理) ← 图像
GPU 路径：GPU(预处理) → 直接写入模型输入 buffer
```

---

# 第五部分：从 preprocess.cu 学到的设计要点

## 1. GPU 预处理 vs CPU 预处理的取舍

不是所有操作都值得搬到 GPU。但当数据已经在 GPU 上（或即将传到 GPU）时，在 GPU 上一步完成比"CPU 处理 → 再传 GPU"更快。

## 2. 一步完成 Letterbox + 归一化 + CHW

传统流水线分多步完成（resize → padding → normalize → transpose），每步都有内存读写。CUDA 内核把所有操作**融合在一个 kernel** 中，从源图像直接写出最终的 NCHW float blob，只读一次源像素、只写一次目标像素。

这种**算子融合（Kernel Fusion）**是 GPU 性能优化的核心思想。

## 3. 线程映射的简洁性

一个线程一个目标像素，逻辑清晰，无分支（除了 padding 判断），缓存友好。这是 CUDA 图像处理内核的标准模式。

## 4. `__restrict__` 的优化提示

告诉编译器指针无别名，允许更激进的指令调度。在带宽受限的内核中，这种提示可能带来 5~10% 的性能提升。
