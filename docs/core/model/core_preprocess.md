# `preprocess.hpp` + `preprocess.cu` 逐行讲解

这两个文件实现了**GPU 端图像预处理**，是当前版本中将预处理从 CPU 迁移到 GPU 的核心模块。

---

# 一、设计动机

旧版预处理完全在 CPU 上完成：

```text
原图(CPU) → cv::resize(CPU) → Letterbox填充(CPU) → blobFromImage(CPU) → H2D拷贝 → GPU推理
```

对于 1920×1080 的输入图像，每帧预处理约消耗 **2~4ms CPU 时间**，包括：

* `cv::resize`：双线性插值缩放 ~1ms
* `cv::Mat::zeros` + `setTo` + `copyTo`：Letterbox 填充 ~0.5ms
* `cv::dnn::blobFromImage`：归一化 + HWC→CHW + 浮点转换 ~1.5ms

新版把所有像素级操作搬到 GPU：

```text
原图(CPU) → H2D拷贝 → GPU kernel(resize + letterbox + normalize + HWC→CHW) → GPU推理
```

CPU 只负责一次 `cudaMemcpyAsync` 上传原始字节（~0.3ms），所有像素处理由 CUDA kernel 完成。

---

# 二、`preprocess.hpp` — 函数声明

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

## 参数详解

| 参数 | 类型 | 含义 |
|------|------|------|
| `src` | `const uint8_t*` | 源图像数据（BGR, uint8, HWC 格式），**设备端指针** |
| `src_w` | `int` | 源图像宽度（像素） |
| `src_h` | `int` | 源图像高度（像素） |
| `src_step` | `int` | 源图像行步长（字节），可能 != `src_w * 3`（OpenCV 有行对齐） |
| `dst` | `float*` | 输出 float buffer（CHW 格式），**设备端指针**，即 TensorRT 的 `buffers[0]` |
| `dst_w` | `int` | 目标宽度（模型输入宽度，如 640） |
| `dst_h` | `int` | 目标高度（模型输入高度，如 640） |
| `scale_inv_x` | `float` | `src_w / new_w`，即 x 方向缩放恢复系数 |
| `scale_inv_y` | `float` | `src_h / new_h`，即 y 方向缩放恢复系数 |
| `new_w` | `int` | 缩放后的图像宽度（不含 padding） |
| `new_h` | `int` | 缩放后的图像高度（不含 padding） |
| `mean` | `const float[3]` | 每通道均值，用于标准化 |
| `std` | `const float[3]` | 每通道标准差，用于标准化 |
| `swapRB` | `bool` | 是否交换 R 和 B 通道（BGR→RGB） |
| `stream` | `cudaStream_t` | CUDA 流，kernel 提交到此流中异步执行 |

---

## 设计要点

### 所有指针都是设备端指针

`src` 和 `dst` 都指向 GPU 显存。调用方需要先通过 `cudaMemcpyAsync` 把源图像从 CPU 传到 GPU。

### `src_step` 的必要性

OpenCV 的 `cv::Mat` 的行步长 `step[0]` 可能不等于 `cols * 3`（有行对齐 padding）。kernel 按实际步长读取，避免越界。

### `scale_inv_x` / `scale_inv_y`

这两个参数是**逆缩放系数**，用于 kernel 内部的反向映射（从目标像素坐标反推源图像坐标）。调用方传入 `src_w / new_w` 而不是 `new_w / src_w`，因为 kernel 内部用乘法比除法更高效。

### `mean` 和 `std` 的控制

* 检测模型：`mean={0,0,0}, std={1,1,1}` → 只做 `/255.0` 归一化
* 分类模型：`mean={0.485,0.456,0.406}, std={0.229,0.224,0.225}` → ImageNet 标准化

这些参数在 `Model` 构造时根据模型类型自动设置，通过成员变量 `mean_[3]` / `std_[3]` 传入。

---

# 三、`preprocess.cu` — CUDA Kernel 实现

> 以下为 `launch_preprocess` 内部 CUDA kernel 的设计逻辑讲解。

---

## Kernel 整体流程

每个 CUDA 线程负责目标图像（模型输入）中的**一个像素**。线程网格覆盖整个 `dst_w × dst_h` 的输出区域。

```text
目标像素 (dx, dy)
    ↓
判断是否在 Letterbox 区域内
    ↓
在有效区域 → 反向映射到源图像坐标 (sx, sy)
    ↓
Bilinear 插值采样源图像
    ↓
BGR→RGB 交换（如果 swapRB=true）
    ↓
÷ 255.0 归一化
    ↓
(x - mean) / std 标准化
    ↓
写入 dst 的 CHW 位置
```

---

## Letterbox 区域判断

```text
┌─────────────────────────────┐
│         padding (114)        │
│  ┌──────────────────────┐   │
│  │                      │   │
│  │   有效缩放图像        │   │
│  │   (new_w × new_h)    │   │
│  │                      │   │
│  └──────────────────────┘   │
│         padding (114)        │
└─────────────────────────────┘
        dst_w × dst_h
```

如果 `dx >= new_w` 或 `dy >= new_h`，该像素处于 padding 区域，直接写入 `(114/255 - mean) / std`（padding 颜色经过相同归一化处理）。

---

## Bilinear 插值

对于有效区域内的像素 `(dx, dy)`：

1. **反向映射**：`sx = dx * scale_inv_x`，`sy = dy * scale_inv_y`
2. **取四个最近邻像素**：`(⌊sx⌋, ⌋sy⌋)` 及其邻居
3. **双线性加权**：根据小数部分计算加权平均

```text
(x0,y0) ──── (x1,y0)
  │   P          │
  │              │
(x0,y1) ──── (x1,y1)

P = (1-wx)*(1-wy)*I(x0,y0) + wx*(1-wy)*I(x1,y0)
  + (1-wx)*wy*I(x0,y1)     + wx*wy*I(x1,y1)
```

其中 `wx = sx - ⌊sx⌋`，`wy = sy - ⌊sy⌋`。

---

## BGR→RGB 交换

```text
如果 swapRB=true：
  输出通道 0 (R) ← 源通道 2 (B)
  输出通道 1 (G) ← 源通道 1 (G)
  输出通道 2 (B) ← 源通道 0 (R)
否则：
  通道顺序不变
```

OpenCV 默认使用 BGR，大多数深度学习模型训练时使用 RGB。`swapRB=true` 完成这个转换。

---

## HWC→CHW 格式转换

```text
源图像(HWC)：pixel[r,g,b, r,g,b, r,g,b, ...]  (行优先)
输出(CHW)：  [RRRRRR..., GGGGGG..., BBBBBB...]  (通道优先)
```

Kernel 在写出时直接按 CHW 索引：

```cpp
dst[0 * dst_h * dst_w + dy * dst_w + dx] = r_channel;
dst[1 * dst_h * dst_w + dy * dst_w + dx] = g_channel;
dst[2 * dst_h * dst_w + dy * dst_w + dx] = b_channel;
```

---

## 归一化 + 标准化

```cpp
float pixel = (raw / 255.0f - mean[c]) / std[c];
```

一步完成归一化和标准化。对于检测模型（`mean=0, std=1`），等价于只做 `/255.0`。

---

# 四、从预处理模块学到的设计要点

## 1. GPU 预处理的收益

| 操作 | CPU 耗时 (1080p) | GPU 耗时 |
|------|-----------------|----------|
| `cv::resize` | ~1ms | - |
| Letterbox 填充 | ~0.5ms | - |
| `blobFromImage` | ~1.5ms | - |
| `cudaMemcpyAsync` (H2D) | ~0.3ms | ~0.3ms |
| `launch_preprocess` kernel | - | ~0.2ms |
| **总计** | **~3.3ms** | **~0.5ms** |

GPU 预处理比 CPU 快约 **6 倍**，且完全不占用 CPU 资源。

## 2. Kernel 设计原则：一个线程一个输出像素

这是 GPU 编程的经典模式：**embarrassingly parallel**。每个线程独立计算一个输出像素，无共享内存、无原子操作、无线程间同步，最大化 GPU 占用率。

## 3. 避免 GPU 内的分支发散

Letterbox padding 区域的判断是**按线程分支**的。同一 warp 中如果有线程在 padding 内、有线程在有效区域外，会导致分支发散（divergence）。但 padding 区域通常只占一小部分（等比例缩放后），影响有限。

## 4. `src_step` 参数防止越界

OpenCV 的行步长可能有 padding（例如 1920×3=5760 但 step=5764）。如果 kernel 按 `src_w * 3` 计算偏移，会读到错误数据甚至越界。`src_step` 参数确保正确寻址。

## 5. 函数指针接口设计

`launch_preprocess` 是一个普通的 C++ 函数声明（不是类方法），实现在 `.cu` 文件中。这种**头文件声明 + CUDA 实现**的分离模式，允许非 CUDA 代码（如 `model.cpp`）通过 `#include "preprocess.hpp"` 调用，而 `.cu` 文件由 nvcc 单独编译。
