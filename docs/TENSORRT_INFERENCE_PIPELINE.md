# TensorRT 推理流水线 (TensorRT Inference Pipeline)

## 1. 模型架构概览

本项目使用 **4 个 TensorRT 引擎** 构成级联检测流水线。所有模型均提前从 PyTorch/ONNX 导出为 `.engine` 文件，运行时直接反序列化加载，无需 ONNX 解析。

| 模型文件 | 输入尺寸 | 用途 | NMS | 输出格式 |
|---|---|---|---|---|
| `robot_only.engine` | 1280×1280×3 | 车辆检测 (car) | YOLO-style + NMS | [boxes, scores, classes] |
| `newarmor.engine` | 192×192×3 | 装甲板检测 (armor) | NMS | [x1,y1,x2,y2,score,class_id] |
| `classify_hku.engine` | 64×64×3 | 装甲板分类 (R1-R4/S) | 无(分类) | [class_logits] |
| `airplane640.engine` | 640×640×3 | 无人机检测 | YOLO-style | [boxes, scores, classes] |

**级联检测策略**：
- **Stage 1**: 在大图 (1280×1280) 上检测所有车辆 —— 全图范围
- **Stage 2**: 对每辆检测到的车，在其 ROI 内运行装甲板检测 (192×192) —— 级联精检
- **Stage 3**: 对每个装甲板 ROI 运行分类模型 (64×64) —— 识别兵种
- **Stage 4**: 异步在图像右半区检测无人机 (640×640) —— 独立低频

---

## 2. Model 类设计

[model.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/model.hpp) / [model.cpp](../src/tensorrt_detect/src/core/model.cpp)

### 2.1 核心数据结构

```cpp
struct Result {
    int idx = 0;             // 类别 ID
    float confidence = 0.0f; // 置信度
    cv::Rect box;            // 边界框（原图坐标）
    int armorColor = 0;      // 装甲板颜色 (1=RED, 2=BLUE)
    cv::Rect car_box{};      // 所属车辆的边界框（装甲板专有）
    cv::Point2f worldPoint{};// 世界坐标（暂存，后续由 PoseNode 真实解算）
    float fps = 0.0f;        // 当前帧率
    bool isDead = false;     // 是否为死亡车辆/装甲板
};
```

### 2.2 Model 生命周期

```
构造函数
  ├── cudaFree(0)                     // 强制初始化 CUDA primary context
  ├── 读取 .engine 文件到内存
  ├── createInferRuntime()            // 创建 TensorRT Runtime
  ├── deserializeCudaEngine()         // 反序列化引擎
  ├── createExecutionContext()        // 创建执行上下文
  ├── 获取 I/O tensor 名称和形状
  ├── cudaMalloc() buffers[2]        // GPU 输入/输出缓冲区
  ├── cudaMallocHost() prob_         // Pinned CPU 后处理缓冲区
  ├── cudaStreamCreate()             // 异步 CUDA 流
  └── 初始化归一化参数                  // Detect: identity, Classify: ImageNet

析构函数
  ├── cudaFree() 所有 GPU 缓冲区
  ├── cudaFreeHost() Pinned 内存
  ├── cudaDestroyTextureObject() 纹理对象
  ├── cudaEventDestroy() / cudaStreamDestroy()
  └── delete context / engine / runtime
```

### 2.3 推理流程

```cpp
bool Model::Detect(const cv::Mat &frame) {
    // 1. GPU 预处理 (H2D + CUDA kernel)
    preprocessing(frame);

    // 2. TensorRT 推理 (异步)
    context->enqueueV3(this->stream);

    // 3. D2H 异步拷贝 + CUDA Event 同步
    cudaMemcpyAsync(prob_, buffers[1], probSize_ * sizeof(float),
                    cudaMemcpyDeviceToHost, this->stream);
    cudaEventRecord(readyEvent_, this->stream);
    cudaEventSynchronize(readyEvent_);

    // 4. CPU 后处理 (解析输出张量)
    postprocessing();
    return !detectResults.empty();
}
```

**异步同步策略**：使用 `cudaEventSynchronize` 而非 `cudaStreamSynchronize`，允许在多个 stream 间精准等待特定事件。

---

## 3. GPU 预处理 (CUDA Preprocessing)

[preprocess.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/preprocess.hpp) / [preprocess.cu](../src/tensorrt_detect/src/core/preprocess.cu)

### 3.1 两条预处理路径

```
原始帧 (BGR 8UC3, CPU memory)
    │
    ├──[路径 A: Texture Accelerated]──┐
    │   cudaMemcpyAsync (pinned H2D)  │                   ← 异步 DMA
    │   cudaCreateTextureObject()     │                   ← 绑定纹理
    │   launch_preprocess_tex()       │ → GPU float NCHW  │ ← 硬件双线性
    │                                 │                   │   插值
    └──[路径 B: Raw Pointer Fallback]─┘
        cudaMemcpyAsync (pinned H2D)                      ← 同 A
        launch_preprocess()           │ → GPU float NCHW  │ ← 软件双线性
                                                                 插值
```

### 3.2 纹理预处理内核 (路径 A — 优先使用)

```cuda
// preprocess.cu:90-131
__global__ void preprocess_kernel_tex(
    cudaTextureObject_t tex,      // 绑定到 gpuInputBuffer8U_ 的纹理对象
    int src_w, int src_h,         // 源尺寸
    float* __restrict__ dst,       // 目标 NCHW float buffer
    int dst_w, int dst_h,         // 目标尺寸
    float scale_inv_x, scale_inv_y, // 缩放因子倒数
    int new_w, int new_h,          // 有效内容区域（letterbox 内）
    float mean0, float mean1, float mean2,
    float std0, float std1, float std2,
    bool swapRB)                   // BGR→RGB
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= dst_w || y >= dst_h) return;

    float4 pixel;
    if (x < new_w && y < new_h) {
        // 硬件双线性插值 tex2D
        // +0.5 offset: CUDA linear-memory 纹理约定，对齐 texel center
        float sx = fminf(fmaxf(x * scale_inv_x + 0.5f, 0.5001f), src_w - 0.5001f);
        float sy = fminf(fmaxf(y * scale_inv_y + 0.5f, 0.5001f), src_h - 0.5001f);
        pixel = tex2D<float4>(tex, sx, sy);  // 硬件插值，一次完成
    } else {
        // Letterbox 填充区域: 填充 pad 值 (114/255)
        pixel.x = pixel.y = pixel.z = 114.0f / 255.0f;
        pixel.w = 0.0f;
    }

    int pixel_idx = y * dst_w + x;
    // BGR→RGB swizzle + 归一化
    for (int c = 0; c < 3; ++c) {
        int src_c = swapRB ? (2 - c) : c;
        float v = src_c == 0 ? pixel.x : (src_c == 1 ? pixel.y : pixel.z);
        v = (v - mean[c]) / stdv[c];
        dst[c * dst_w * dst_h + pixel_idx] = v;  // NCHW 布局
    }
}
```

### 3.3 纹理对象创建

```cpp
// model.cpp:230-253 — 纹理对象创建
cudaResourceDesc resDesc = {};
resDesc.resType = cudaResourceTypePitch2D;
resDesc.res.pitch2D.devPtr = gpuInputBuffer8U_;     // GPU 上的 BGRA 8-bit 数据
resDesc.res.pitch2D.desc = cudaCreateChannelDesc(8, 8, 8, 0,
                            cudaChannelFormatKindUnsigned);
resDesc.res.pitch2D.width = img_w;
resDesc.res.pitch2D.height = img_h;
resDesc.res.pitch2D.pitchInBytes = frame.step[0];

cudaTextureDesc texDesc = {};
texDesc.addressMode[0] = cudaAddressModeClamp;       // 边界钳制
texDesc.addressMode[1] = cudaAddressModeClamp;
texDesc.filterMode = cudaFilterModeLinear;           // 线性插值
texDesc.readMode = cudaReadModeNormalizedFloat;       // 返回 [0,255]→[0,1]
texDesc.normalizedCoords = 0;                        // 使用像素坐标

cudaCreateTextureObject(&inputTex_, &resDesc, &texDesc, nullptr);
```

**为什么优先使用纹理路径**：
- `tex2D<float4>` 硬件双线性插值：4 个像素值读取 + 加权混合在**单个 SM 指令**内完成
- 相比手动 4× 内存加载 + 浮点乘加约 **2-3× 更快**
- 纹理缓存在 L1 中优化了 2D 空间局部性

### 3.4 Bitstream 归一化差异

```cpp
// model.cpp:103-109
if (this->output_h == 1 || this->output_w == 1) {
    // Classify 模型：ImageNet 归一化
    mean_[0] = 0.485f; mean_[1] = 0.456f; mean_[2] = 0.406f;
    std_[0]  = 0.229f; std_[1]  = 0.224f; std_[2]  = 0.225f;
} else {
    // Detect 模型：仅 1/255 归一化
    mean_[0] = 0.0f; mean_[1] = 0.0f; mean_[2] = 0.0f;
    std_[0]  = 1.0f; std_[1]  = 1.0f; std_[2]  = 1.0f;
}
```

---

## 4. GPU 缓冲区管理

### 4.1 按需增长策略

```cpp
// model.cpp:193-218 — Pinned CPU buffer 和 Device buffer 按需增长
void Model::preprocessing(const cv::Mat &frame) {
    size_t imgBytes = frame.step[0] * frame.rows;

    // Pinned CPU 暂存缓冲区：容量不足才重新分配
    if (hInputCapacity_ < imgBytes) {
        if (hInputBuffer8U_) cudaFreeHost(hInputBuffer8U_);
        cudaMallocHost(&hInputBuffer8U_, imgBytes);  // Pinned memory for async DMA
        hInputCapacity_ = imgBytes;
    }
    memcpy(hInputBuffer8U_, frame.data, imgBytes);

    // GPU 设备缓冲区：同样按需增长
    if (gpuInputCapacity_ < imgBytes) {
        if (gpuInputBuffer8U_) cudaFree(gpuInputBuffer8U_);
        cudaMalloc(&gpuInputBuffer8U_, imgBytes);
        gpuInputCapacity_ = imgBytes;
    }
    cudaMemcpyAsync(gpuInputBuffer8U_, hInputBuffer8U_, imgBytes,
                    cudaMemcpyHostToDevice, this->stream);
    // ...
}
```

**为什么使用 Pinned Memory**：
- `cudaMallocHost` 分配的页面锁定内存允许 GPU DMA 引擎直接访问
- `cudaMemcpyAsync` 从 pinned memory 到 device 可实现真正的异步传输（非阻塞 CPU）
- 普通 `new`/`malloc` 内存需要先拷贝到内部 pinned staging buffer，增加延时

### 4.2 缓冲生命周期

```
┌──────────────────────────────────────────────────┐
│ Model 对象                                        │
│ ┌──────────────────┐ ┌─────────────────────────┐ │
│ │ buffers[0]       │ │ gpuInputBuffer8U_       │ │
│ │ (固定大小)        │ │ (按需增长，≤frame bytes) │ │
│ │ Model I/O tensor │ │ 源图像暂存              │ │
│ │ input_h×input_w  │ │                         │ │
│ │ ×3×sizeof(float) │ │                         │ │
│ └──────────────────┘ └─────────────────────────┘ │
│ ┌──────────────────┐ ┌─────────────────────────┐ │
│ │ buffers[1]       │ │ hInputBuffer8U_         │ │
│ │ (固定大小)        │ │ Pinned host memory      │ │
│ │ output_h×output_w│ │ 异步 H2D 传输源          │ │
│ │ ×sizeof(float)   │ │                         │ │
│ └──────────────────┘ └─────────────────────────┘ │
│ ┌──────────────────┐ ┌─────────────────────────┐ │
│ │ prob_            │ │ inputTex_               │ │
│ │ Pinned host       │ │ cudaTextureObject_t     │ │
│ │ D2H 异步传输目标  │ │ 硬件双线性插值绑定       │ │
│ └──────────────────┘ └─────────────────────────┘ │
└──────────────────────────────────────────────────┘
```

---

## 5. 后处理解析

### 5.1 NMS 模型输出解析

```cpp
// model.cpp:293-328 — 标准 NMS 输出 (robot_only, newarmor)
// 输出格式: [N×6 or 6×N] where columns are [x1, y1, x2, y2, score, class_id]
if (this->isNMS) {
    // 自动检测行列格式 (行优先或列优先)
    bool is_transposed = (det_output.rows < det_output.cols);
    int num_boxes = is_transposed ? det_output.cols : det_output.rows;

    for (int i = 0; i < num_boxes; ++i) {
        // 提取 score, 若 ≤ 阈值则跳过
        score = det_output.at<float>(4, i);   // or (i, 4)
        if (score <= this->scoreThreshold) continue;
        // 边界框坐标变换回原图
        box.x = x1 * rx;  // rx = 原图宽度 / 缩放后宽度
        box.y = y1 * ry;
        box.width = (x2 - x1) * rx;
        box.height = (y2 - y1) * ry;
        results.emplace_back(Result{class_id, score, box});
    }
}
```

### 5.2 YOLO-style 非 NMS 输出解析

```cpp
// model.cpp:331-403 — YOLO-style 原始输出 (airplane640)
// 输出格式: [N×M or M×N] where first 4 columns are [cx, cy, w, h]
else {
    // 提取 anchor 数量 × 属性数量
    // 对每个 anchor: 解析 cx/cy/w/h + class probabilities
    // NMS 后处理 (OpenCV cv::dnn::NMSBoxes)

    std::vector<int> indexes;
    cv::dnn::NMSBoxes(boxes, confidences, scoreThreshold, nmsThreshold, indexes);
    for (int idx : indexes) {
        results.emplace_back(Result{classIds[idx], confidences[idx], boxes[idx]});
    }
}
```

### 5.3 分类模型输出

```cpp
// model.cpp:445-458 — Classify 模型
int Model::predictClass(const cv::Mat &roi) {
    preprocessing(roi);         // ImageNet 归一化
    context->enqueueV3(stream); // 推理
    // D2H + sync
    cudaMemcpyAsync(prob_, buffers[1], probSize_ * sizeof(float), ...);
    cudaEventSynchronize(readyEvent_);
    // Argmax
    return std::max_element(data, data + probSize_) - data;
}
```

---

## 6. 流水线编排 (DetectPipeline)

[pipeline.hpp](../src/tensorrt_detect/include/tensorrt_detect/core/pipeline.hpp) / [pipeline.cpp](../src/tensorrt_detect/src/core/pipeline.cpp)

### 6.1 Stage 1: 车辆检测

```cpp
// pipeline.cpp:49-60
std::vector<Result> DetectPipeline::runDetect(const cv::Mat& frame) {
    detectModel_.Detect(frame);     // TensorRT 推理
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);
    std::vector<Result> filtered;
    for (const auto& det : detectModel_.detectResults) {
        cv::Rect safeBox = det.box & imgBound;  // 裁剪到图像边界
        if (safeBox.width >= cfg_.model.minRoiSize &&
            safeBox.height >= cfg_.model.minRoiSize) {
            filtered.push_back(det);            // 过滤过小的 ROI
        }
    }
    return filtered;
}
```

### 6.2 Stage 2a: 装甲板检测

```cpp
// pipeline.cpp:62-107 — 级联检测：每车独立运行 armorDetector_
for (const auto& det : detections) {
    cv::Rect roi = det.box & imgBound;              // 车辆 ROI
    if (!armorDetector_.Detect(frame(roi))) continue; // 在 car ROI 内检测

    // 取最高置信度装甲板
    auto maxArmor = std::max_element(armorDetector_.detectResults.begin(),
                                      armorDetector_.detectResults.end(),
        [](const Result& a, const Result& b) {
            return a.confidence < b.confidence;
        });
    Result armor = *maxArmor;
    // raw_id=0 → dead armor, raw_id>0 → armorColor
    armor.isDead = (raw_id == DEAD_ARMOR_ID);
    armor.box.x += roi.x;  // 偏移回原图坐标
    armor.box.y += roi.y;
    armor.car_box = det.box;  // 关联父车辆
}
```

### 6.3 Stage 2b: 前哨站检测

```cpp
// pipeline.cpp:109-169 — ROI 内独立检测 + 死亡判定
if (armorDetector_.Detect(frame(safeOutpostRoi))) {
    // 找到置信度 > outpostScoreThreshold 的最佳结果
    if (hasValidDetection) {
        outpostMissCount_ = 0;       // 重置丢失计数
        outpostIsDead_ = false;
        bestResult.idx = robot_id::OUTPOST;  // idx=7
    } else {
        outpostMissCount_++;
        if (outpostMissCount_ >= cfg_.model.outpostMissThreshold) {
            outpostIsDead_ = true;   // 连续 N 帧未检测到 → 死亡
        }
    }
}
```

### 6.4 Stage 3: 装甲板分类

```cpp
// pipeline.cpp:172-197 — 分类 + 跳过死亡装甲板
void DetectPipeline::runClassify(const cv::Mat& frame,
                                  std::vector<Result>& detections) {
    for (auto& armor : detections) {
        if (armor.isDead) continue;  // 死亡装甲板保持 ARMOR 类型

        cv::Mat armorROI = frame(armor.box);
        int raw_id = classifyModel_.predictClass(armorROI); // CNN 分类

        if (raw_id == 4)      armor.idx = 6;  // → SENTRY
        else if (raw_id >= 0 && raw_id <= 3) armor.idx = raw_id + 2; // → R2-R4
    }
}
```

**类别映射**：

| raw_id (分类模型输出) | 实际 class_id | 含义 |
|---|---|---|
| 0 | 2 | R1 (1号机器人) |
| 1 | 3 | R2 |
| 2 | 4 | R3 |
| 3 | 5 | R4 |
| 4 | 6 | Sentry (哨兵) |

### 6.5 Stage 4: 异步无人机检测

```cpp
// pipeline.cpp:304-341 — 独立检测线程
void DetectPipeline::airplaneThreadLoop() {
    while (!stopThread_) {
        // wait: 有新的右半帧 OR 超时 100ms OR stop
        airplaneCv_.wait_for(lock, 100ms, ...);

        if (hasNewFrame) {
            airplaneModel_->Detect(frame);     // 在右半区检测
            lastAirplaneMs_ = elapsedMs(ta0, ta1);

            for (auto& res : results) {
                res.idx = robot_id::AIRPLANE;  // idx=8
                res.box.x += xOffset;          // 偏移回原图坐标
            }
            cachedAirplaneResults_ = std::move(results);
        }
        // 低频控制：间隔 airplaneIntervalMs_ (默认 33ms)
        std::this_thread::sleep_for(std::chrono::milliseconds(airplaneIntervalMs_));
    }
}
```

**异步设计要点**：
- 主线程 `process()` 调用 `airplaneCv_.notify_one()`，传递右半帧副本 → 不阻塞
- 异步线程 `airplaneCv_.wait_for()` 接收帧后独立推理
- 结果存入 `cachedAirplaneResults_`（互斥保护），主线程非阻塞合并
- `airplaneIntervalMs_` 控制检测频率，避免 GPU 争抢

---

## 7. 时序统计

```cpp
// pipeline.cpp:42-47 — 耗时统计工具
static inline double elapsedMs(
    const std::chrono::steady_clock::time_point& t0,
    const std::chrono::steady_clock::time_point& t1) {
    return std::chrono::duration<double, std::milli>(t1 - t0).count();
}

// pipeline.cpp:216-277 — process() 内的时序打点
auto t0 = std::chrono::steady_clock::now();     // 开始
// ... 无人机帧传递 ...
auto t1 = std::chrono::steady_clock::now();
auto cars  = runDetect(frame);                   // Stage 1
auto t2 = std::chrono::steady_clock::now();
auto armors  = runArmorDetect(frame, cars);     // Stage 2a
auto outposts = detectOutpost(frame);            // Stage 2b
auto t3 = std::chrono::steady_clock::now();
runClassify(frame, armors);                     // Stage 3
auto t4 = std::chrono::steady_clock::now();
// ... merge + publish ...
auto t6 = std::chrono::steady_clock::now();

// 发布 PipelineTiming 消息：
// car_ms = t2-t1, armor_ms = armorDetect 内部计时,
// cls_ms = t4-t3, outpost_ms = outpost 内部计时,
// airplane_ms = 异步线程计时, total_ms = t6-t0
```

---

## 8. CUDA 多线程安全

```cpp
// model.hpp:14-19 — 全局 CUDA 互斥锁
namespace cuda_guard {
    inline std::mutex& getCudaMutex() {
        static std::mutex mtx;
        return mtx;
    }
}
```

**使用场景**：
1. `Raycaster::pixelToWorld()` / `pixelToWorldBatch()` 中：`lock_guard` 保护 Open3D `CastRays` 调用（内部使用 CUDA）
2. 所有 `Model::Detect()` 调用：得益于单线程容器，TensorRT 推理天然串行；额外互斥保护 Open3D

**为什么需要**：Open3D 的 `RaycastingScene::CastRays()` 和 TensorRT 的 `enqueueV3()` 都在底��使用 CUDA。在单线程容器中，它们顺序执行，但如果未来启用多线程，这个互斥锁保证只有一个 CUDA 操作在运行，防止 Driver API 的并发冲突。

---

## 9. 推理性能特征

| 阶段 | 模型 | 输入 | 典型耗时 | 说明 |
|---|---|---|---|---|
| Preprocess | CUDA Kernel | ~2MP | ~0.3-0.5ms | Texture 路径，含 H2D |
| Car Detect | robot_only | 1280×1280 | ~3-5ms | 最重的单步 |
| Armor Detect × N | newarmor | 192×192 each | ~1-3ms total | N=检测到的车辆数 |
| Armor Classify × M | classify_hku | 64×64 each | ~0.5-1ms total | N ≈ 装甲板数 |
| Outpost Detect | newarmor | ~ROI | ~0.5-1ms | 仅 ROI 范围 |
| Airplane Detect | airplane640 | 640×640 | ~2-4ms | 异步，不阻塞主流程 |
| **端到端总延时** | — | — | ~8-15ms | 主流程不含异步 |
