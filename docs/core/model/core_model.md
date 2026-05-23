# `model.hpp` + `model.cpp` 逐行讲解

这两份文件是项目的**推理引擎核心**，负责 TensorRT 模型的加载、前处理、推理、后处理。整个检测系统的速度和精度，最终都取决于这一层的实现质量。

> **版本变更说明**：当前版本相比旧版有重大变化——预处理从 CPU 全部迁移到 GPU（CUDA kernel）、推理 API 从 `executeV2` 升级到 `enqueueV3`、新增批量推理（Batch Inference）、使用 Pinned Memory 优化 D2H 拷贝、引入全局 CUDA 互斥锁解决多线程并发安全问题。

---

# 第一部分：`model.hpp`

## 一、头文件保护

```cpp
#ifndef __MODEL_HPP__
#define __MODEL_HPP__
```

经典的 C/C++ **头文件包含保护（Include Guard）**。作用是防止同一个头文件被重复包含导致编译错误。

---

## 二、引入的头文件

```cpp
#include <iostream>
#include <vector>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
```

---

### `<iostream>` / `<vector>`

标准 C++ 库：输入输出流、动态数组容器。

---

### `<mutex>`（新增）

C++11 互斥锁。用于 `cuda_guard` 命名空间中的全局 CUDA 互斥锁，解决 `component_container_mt` 多线程并发 CUDA 操作导致的 SIGSEGV 问题。

---

### `<opencv2/opencv.hpp>`

OpenCV 全功能头文件。这里主要用到：

* `cv::Mat`：图像矩阵
* `cv::Rect`：矩形框
* `cv::Point2f`：二维浮点坐标
* `cv::resize`、`cv::dnn::NMSBoxes` 等

---

### `<opencv2/dnn.hpp>`

OpenCV 的深度学习模块。主要借用 NMS 后处理：

* `cv::dnn::NMSBoxes`：非极大值抑制

注意：旧版还用 `cv::dnn::blobFromImage` 做预处理，当前版本预处理已迁移到 GPU（CUDA kernel），但 `preprocessSingle` 方法中仍保留 CPU 路径作为 fallback。

---

### `<NvInfer.h>`

**NVIDIA TensorRT 的核心头文件**。它定义了：

* `nvinfer1::IRuntime`：运行时，负责反序列化引擎
* `nvinfer1::ICudaEngine`：推理引擎，包含网络结构和优化后的计算图
* `nvinfer1::IExecutionContext`：执行上下文，负责实际推理
* `nvinfer1::ILogger`：日志接口
* `nvinfer1::Dims`、`nvinfer1::TensorIOMode`、`nvinfer1::Dims4` 等数据结构

---

### `<cuda_runtime_api.h>`

CUDA 运行时 API。这里用到：

* `cudaMalloc` / `cudaFree`：GPU 显存分配/释放
* `cudaMallocHost` / `cudaFreeHost`：**Pinned（页锁定）内存**分配/释放（新增）
* `cudaMemcpyAsync`：异步内存拷贝（Host ↔ Device）
* `cudaStreamCreate` / `cudaStreamDestroy`：CUDA 流管理
* `cudaStreamSynchronize`：流同步
* `cudaFree(0)`：触发 CUDA primary context 初始化

---

## 三、全局 CUDA 互斥锁（新增）

```cpp
namespace cuda_guard {
    inline std::mutex& getCudaMutex() {
        static std::mutex mtx;
        return mtx;
    }
}
```

### 为什么需要这个？

当节点运行在 `component_container_mt`（ROS2 多线程容器）中时，多个节点组件共享同一进程。如果 `DetectNode` 和 `PoseNode`（使用 Open3D 的 raycasting）**同时执行 CUDA 操作**，会导致 CUDA primary context 竞争，引发 SIGSEGV 崩溃。

这个全局互斥锁序列化进程内所有 CUDA 操作，从根本上消除并发问题。

### `inline` + `static` 局部变量

* `inline`：C++17，允许多个翻译单元包含同一头文件而不产生链接冲突
* `static std::mutex mtx`：函数局部静态变量，保证只初始化一次（线程安全，C++11 magic statics）

---

## 四、`Result` 结构体

```cpp
struct Result
{
    int idx = 0;  
    float confidence = 0.0f; 
    cv::Rect box; 
    int armorColor = 0;
    cv::Rect car_box{};
    cv::Point2f worldPoint{};
    float fps = 0.0f;
    bool isDead = false;
};
```

这是整个项目**最核心的数据结构**。从检测到可视化到 ROS2 消息，几乎所有模块都在传递 `Result` 或它的变体。

---

### 字段设计解读

| 字段 | 类型 | 含义 |
|------|------|------|
| `idx` | `int` | 类别索引。0=CAR, 1=ARMOR, 2=R1, ..., 6=S, 7=OUTPOST, 8=AIRPLANE |
| `confidence` | `float` | 模型输出的置信度分数，0~1 |
| `box` | `cv::Rect` | 当前目标的检测框。对装甲板检测来说是装甲板框 |
| `armorColor` | `int` | 队伍/颜色 ID。参考 `robot_id.hpp`：UNKNOWN=0, RED=1, BLUE=2 |
| `car_box` | `cv::Rect` | 车辆整体检测框。由第一阶段目标检测产生 |
| `worldPoint` | `cv::Point2f` | 图像上的"世界坐标点"（实际是像素坐标，供后续位姿解算用） |
| `fps` | `float` | 当前处理帧率，主要用于传递显示 |
| `isDead` | `bool` | **死亡装甲板标志**。`true` 表示该装甲板属于死亡车辆，不再进入第三层分类 |

---

### `isDead` 字段的设计意义

`isDead` 是后续添加的关键字段，用于**显式区分死亡装甲板和存活装甲板**。

原始代码中，死亡状态被隐式编码在 `armorColor == 0` 中，这导致两个问题：
1. `armorColor` 同时承担"颜色/队伍"和"死亡状态"两种语义，容易混淆
2. 下游节点无法准确判断一个 `armorColor == 0` 的目标是"未识别颜色的存活装甲板"还是"死亡装甲板"

引入独立的 `isDead` 标志后：
- `isDead == true` → 死亡装甲板，不进入分类，在地图上显示为黑色 `"dead"`
- `isDead == false` → 存活装甲板，正常进入分类，在地图上按 `armorColor` 显示红/蓝

---

## 五、`Model` 类声明

```cpp
class Model
{
```

`Model` 是一个**TensorRT 推理封装类**。每个 `Model` 实例对应一个加载到 GPU 上的 TensorRT 引擎。

在当前项目里，同时存在**四个** `Model` 实例：

1. `detectModel_`：第一阶段目标检测（检测车辆）
2. `armorDetector_`：第二阶段装甲板检测（在车辆 ROI 内检测装甲板）
3. `classifyModel_`：第三阶段分类（识别装甲板属于哪个机器人）
4. `airplaneDetector_`：**第四阶段无人机检测**（新增，在独立线程/间隔中检测无人机）

---

## 六、私有成员变量

```cpp
private:
    int inputSize;        
    float scoreThreshold; 
    float nmsThreshold;   
    bool isNMS;
```

---

### `inputSize`

模型输入图像的边长。比如 YOLO 通常用 640×640，分类模型可能用 224×224。

---

### `scoreThreshold`

置信度过滤阈值。模型输出成千上万个候选框，只有 confidence > scoreThreshold 的才保留。

---

### `nmsThreshold`

NMS（Non-Maximum Suppression）的 IoU 阈值。通常设 0.45~0.65。

---

### `isNMS`

控制后处理流程的分支：

* `true`：模型已经做了内置 NMS（如 YOLOv8 的 end-to-end 输出）
* `false`：模型输出原始 anchor 预测，需要手动做 NMS

---

## 七、TensorRT 核心对象

```cpp
    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    cudaStream_t stream = nullptr;
    void *buffers[2] = {nullptr, nullptr};
```

---

### TensorRT 推理三件套

```text
IRuntime ──反序列化──→ ICudaEngine ──创建──→ IExecutionContext
```

* `IRuntime`：TensorRT 运行时，负责反序列化引擎
* `ICudaEngine`：优化后的推理引擎，线程安全只读对象
* `IExecutionContext`：执行上下文，负责实际推理

### `stream`

CUDA 流。所有 GPU 操作（内存拷贝、推理、预处理 kernel）都提交到这个流中异步执行。

### `buffers[2]`

GPU 显存指针数组。`buffers[0]` 指向输入 buffer，`buffers[1]` 指向输出 buffer。

---

## 八、Pinned Memory 缓冲区（新增）

```cpp
    // Pinned host buffers for true async memcpy
    float* prob_ = nullptr;
    size_t probSize_ = 0;
    float* hOutput_ = nullptr;
    size_t hOutputSize_ = 0;
```

---

### 为什么用 Pinned Memory？

旧版使用 `std::vector<float> prob`（普通堆内存）作为 D2H 拷贝的目标。问题是：

* 普通堆内存可能被操作系统换出到磁盘（swap）
* `cudaMemcpyAsync` 遇到 pageable memory 时，CUDA 驱动会**隐式同步**，退化成同步拷贝

新版使用 `cudaMallocHost` 分配 **Pinned（页锁定）内存**，保证：

* 内存始终在物理 RAM 中，不会被 swap
* `cudaMemcpyAsync` 可以真正做到异步，CPU 和 GPU 并行工作

### `prob_` vs `hOutput_`

* `prob_`：单帧推理的 D2H 输出缓冲区
* `hOutput_`：**批量推理**的 D2H 输出缓冲区（新增），按需增长

---

## 九、批量推理缓冲区（新增）

```cpp
    // Pre-allocated batch buffers (grow-on-demand)
    void* batchInputBuffer_ = nullptr;
    void* batchOutputBuffer_ = nullptr;
    size_t batchInputCapacity_ = 0;
    size_t batchOutputCapacity_ = 0;
```

批量推理时，多个 ROI 图像的预处理结果拼接成一个大 tensor 一次性送入 GPU。

* `batchInputBuffer_`：拼接后的 batch 输入 buffer（GPU 显存）
* `batchOutputBuffer_`：batch 输出 buffer（GPU 显存）
* `batchInputCapacity_` / `batchOutputCapacity_`：当前已分配容量（字节），按需增长

### Grow-on-demand 模式

```cpp
void Model::ensureBatchBuffers(size_t inputBytes, size_t outputBytes);
```

当新 batch 需要的 buffer 大于当前容量时，先 `cudaFree` 再 `cudaMalloc` 更大的 buffer。容量只会增长，不会缩小，避免频繁分配。

---

## 十、GPU 预处理缓冲区（新增）

```cpp
    // GPU preprocessing buffers (grow-on-demand)
    void* gpuInputBuffer8U_ = nullptr;
    size_t gpuInputCapacity_ = 0;

    // Batch GPU preprocessing staging buffer
    void* gpuBatchInput8U_ = nullptr;
    size_t gpuBatchInputCapacity_ = 0;
```

---

### `gpuInputBuffer8U_`

单帧推理时，原始图像（uint8 BGR）从 CPU 上传到 GPU 的 staging buffer。`launch_preprocess` CUDA kernel 从这个 buffer 读取原始图像，完成 Letterbox + 归一化 + HWC→CHW 转换，直接写入 `buffers[0]`。

### `gpuBatchInput8U_`

批量推理时，所有 ROI 图像按顺序上传到这个 buffer，然后对每个 ROI 调用 `launch_preprocess`。

两者都是 grow-on-demand 模式，避免频繁分配。

---

## 十一、Tensor 名称和归一化参数（新增）

```cpp
    // Tensor names stored for batch operations
    std::string inputName_;
    std::string outputName_;

    // Normalization params (ImageNet for classify, identity for detect)
    float mean_[3] = {0.0f, 0.0f, 0.0f};
    float std_[3]  = {1.0f, 1.0f, 1.0f};
```

---

### `inputName_` / `outputName_`

TensorRT 10.x 的 `enqueueV3` API 需要用 tensor 名称来设置输入输出地址。构造时自动从 engine 中推断，不硬编码。

### `mean_[3]` / `std_[3]`

GPU 预处理 kernel 使用的归一化参数：

* 检测模型：`mean={0,0,0}, std={1,1,1}`（即不做 ImageNet 标准化）
* 分类模型：`mean={0.485,0.456,0.406}, std={0.229,0.224,0.225}`（ImageNet 标准化）

构造时根据 `output_h == 1 || output_w == 1` 判断模型类型，自动设置。

---

## 十二、输入输出维度

```cpp
    int input_h = 0, input_w = 0;
    int output_h = 0, output_w = 0;
    float rx = 0.0f, ry = 0.0f;
```

* `input_h` / `input_w`：模型输入 tensor 的高度和宽度
* `output_h` / `output_w`：模型输出 tensor 的维度
* `rx` / `ry`：缩放恢复系数（原图尺寸 / 缩放后尺寸），后处理时坐标映射回原图

---

## 十三、内部方法（重构）

```cpp
    cv::Mat resizeFrame;

    void preprocessing(const cv::Mat &frame);
    void postprocessing();

    cv::Mat preprocessSingle(const cv::Mat& frame, float& rx, float& ry);
    std::vector<Result> postprocessSingle(const cv::Mat& det_output, float rx, float ry);
    std::vector<std::vector<Result>> postprocessBatch(const float* outputData, int batchSize, const std::vector<float>& rxs, const std::vector<float>& rys);
    void ensureBatchBuffers(size_t inputBytes, size_t outputBytes);
```

---

### 方法拆分说明

旧版只有 `preprocessing` 和 `postprocessing` 两个内部方法，所有逻辑混在一起。

新版做了清晰拆分：

| 方法 | 职责 | 使用场景 |
|------|------|---------|
| `preprocessing` | 单帧 GPU 预处理 | `Detect()`、`predictClass()` |
| `postprocessing` | 单帧 D2H + 后处理 | `Detect()` |
| `preprocessSingle` | 单帧 **CPU** 预处理（返回 blob Mat） | `postprocessSingle` 的前置步骤 |
| `postprocessSingle` | 纯后处理（解析 Mat 输出） | `postprocessBatch`、`DetectBatchSlow` |
| `postprocessBatch` | 批量后处理 | `DetectBatch` |
| `ensureBatchBuffers` | 确保 batch buffer 足够大 | `predictClassBatch`、`DetectBatch` |

注意：`preprocessing`（GPU 路径）和 `preprocessSingle`（CPU 路径）是**两套独立的预处理实现**。`preprocessSingle` 保留给 `preprocessSingle`/`postprocessSingle` 的 CPU fallback 路径使用。

---

## 十四、公共接口（扩展）

```cpp
public:
    enum class ModelType
    {
    DETECT,
    CLASSIFY,
    UNKNOWN
    };
    std::vector<Result> detectResults;
    ModelType modelType;

    Model(const std::string modelPath, const int &inputSize,
          const float &scoreThreshold, const float &nmsThreshold,
          const bool isNMS = true,
          const ModelType modelType = ModelType::DETECT);
    ~Model();

    int predictClass(const cv::Mat &roi);
    cv::Rect roi;

    bool Detect(const cv::Mat &frame);

    // Batch methods (新增)
    std::vector<int> predictClassBatchSlow(const std::vector<cv::Mat>& rois);
    std::vector<int> predictClassBatch(const std::vector<cv::Mat>& rois);
    std::vector<std::vector<Result>> DetectBatchSlow(const std::vector<cv::Mat>& rois);
    std::vector<std::vector<Result>> DetectBatch(const std::vector<cv::Mat>& rois);
```

---

### 新增的四个批量方法

| 方法 | 功能 | 实现方式 |
|------|------|---------|
| `predictClassBatchSlow` | 批量分类（低效版） | 循环调用 `predictClass`，逐帧推理 |
| `predictClassBatch` | 批量分类（高效版） | 多帧拼成 batch tensor，一次推理 |
| `DetectBatchSlow` | 批量检测（低效版） | 循环调用 `Detect`，逐帧推理 |
| `DetectBatch` | 批量检测（高效版） | 多帧拼成 batch tensor，一次推理 |

#### 批量推理的价值

当 `classifyModel_` 需要对同一帧中的 5 个装甲板做分类时：

* **逐帧推理**：5 次 `predictClass` = 5 次 H2D + 5 次推理 + 5 次 D2H
* **批量推理**：1 次 `predictClassBatch` = 1 次 H2D + 1 次推理（batch=5）+ 1 次 D2H

批量推理减少了 **4 次 GPU kernel launch 开销**和 **4 次 D2H 同步等待**，在分类模型推理密集时收益显著。

#### 动态 Batch 检测

`predictClassBatch` 和 `DetectBatch` 会检查 engine 的输入维度是否有动态 batch（`d[0] == -1`）：

* 有动态 batch → 拼接多个样本，一次 `enqueueV3` 完成
* 无动态 batch → 退化到 Slow 路径（逐帧推理）

---

# 第二部分：`model.cpp`

## 一、Logger 类（不变）

```cpp
class Logger : public nvinfer1::ILogger
{
    void log(Severity severity, const char *msg) noexcept override
    {
        if (severity <= Severity::kINFO)
            std::cout << msg << std::endl;
    }
} gLogger;
```

---

### 不变的部分

* `noexcept override`：重写虚函数，承诺不抛异常
* 日志级别过滤：只输出 `kINFO` 及以上
* `gLogger`：全局单例，所有 `Model` 实例共用

---

## 二、构造函数（重大变更）

```cpp
Model::Model(const std::string modelPath, const int &inputSize,
             const float &scoreThreshold, const float &nmsThreshold,
             const bool isNMS, const ModelType modelType)
{
    this->inputSize = inputSize;
    this->scoreThreshold = scoreThreshold;
    this->nmsThreshold = nmsThreshold;
    this->isNMS = isNMS;
    this->modelType = modelType;
```

---

### 变更一：CUDA Primary Context 提前初始化

```cpp
    cudaError_t initErr = cudaFree(0);
    if (initErr != cudaSuccess) {
        throw std::runtime_error(std::string("CUDA 初始化失败: ") + cudaGetErrorString(initErr));
    }
```

`cudaFree(0)` 不释放任何东西，但会**触发 CUDA primary context 的创建**。

为什么要在构造函数开头做这个？

当多个节点运行在同一个 `component_container_mt` 进程中时，如果两个线程同时首次访问 CUDA（比如 `DetectNode` 和 `PoseNode`），会并发初始化 CUDA primary context，导致 SIGSEGV。

通过在每个 `Model` 构造时**提前初始化**，确保 CUDA context 在单线程环境下安全创建。

### 变更二：CUDA 设备检测

```cpp
    int deviceCount = 0;
    cudaError_t cudaErr = cudaGetDeviceCount(&deviceCount);
    if (cudaErr != cudaSuccess || deviceCount == 0) {
        throw std::runtime_error("没有可用的 CUDA 设备，无法加载 TensorRT 模型");
    }
```

旧版没有这个检查。如果在没有 GPU 的机器上运行，会直接 SIGSEGV 崩溃。新版提前检测，给出明确的错误信息。

### 变更三：错误处理从 `assert` 改为 `throw`

```cpp
    this->runtime = nvinfer1::createInferRuntime(gLogger);
    if (!this->runtime) {
        throw std::runtime_error("TensorRT runtime 创建失败，请检查 CUDA 驱动和 GPU 可用性");
    }

    this->engine = this->runtime->deserializeCudaEngine(engineData.data(), fsize);
    if (!this->engine) {
        throw std::runtime_error("TensorRT engine 反序列化失败，请检查模型文件是否有效");
    }

    this->context = this->engine->createExecutionContext();
    if (!this->context) {
        throw std::runtime_error("TensorRT execution context 创建失败");
    }
```

旧版用 `assert`，在 Release 编译（`-DNDEBUG`）时会被编译器消除，导致错误被静默忽略。

新版改用 `throw std::runtime_error`，无论编译模式都能保证错误被捕获，且上层 `try/catch` 可以给出有意义的错误信息。

---

### 变更四：Pinned Memory 分配

```cpp
    cudaMalloc(&(this->buffers[0]), this->input_h * this->input_w * 3 * sizeof(float));
    cudaMalloc(&(this->buffers[1]), this->output_h * this->output_w * sizeof(float));
    this->probSize_ = this->output_h * this->output_w;
    cudaMallocHost(&(this->prob_), this->probSize_ * sizeof(float));
    cudaStreamCreate(&(this->stream));
```

旧版：
```cpp
this->prob.resize(this->output_h * this->output_w);
```

新版用 `cudaMallocHost` 分配 Pinned Memory，使 `cudaMemcpyAsync` 真正异步。

---

### 变更五：归一化参数初始化

```cpp
    // Initialize normalization params
    if (this->output_h == 1 || this->output_w == 1) {
        mean_[0] = 0.485f; mean_[1] = 0.456f; mean_[2] = 0.406f;
        std_[0]  = 0.229f; std_[1]  = 0.224f; std_[2]  = 0.225f;
    } else {
        mean_[0] = 0.0f; mean_[1] = 0.0f; mean_[2] = 0.0f;
        std_[0]  = 1.0f; std_[1]  = 1.0f; std_[2]  = 1.0f;
    }
```

把归一化参数从预处理函数中的局部变量提升为**成员变量**，因为 GPU 预处理 kernel 需要在运行时读取这些参数。

---

## 三、析构函数（扩展）

```cpp
Model::~Model()
{
    for (auto &buffer : this->buffers)
    {
        if (buffer)
            cudaFree(buffer);
    }
    if (batchInputBuffer_)
        cudaFree(batchInputBuffer_);
    if (batchOutputBuffer_)
        cudaFree(batchOutputBuffer_);
    if (gpuInputBuffer8U_)
        cudaFree(gpuInputBuffer8U_);
    if (gpuBatchInput8U_)
        cudaFree(gpuBatchInput8U_);
    if (prob_)
        cudaFreeHost(prob_);
    if (hOutput_)
        cudaFreeHost(hOutput_);
    if (this->context)
        delete this->context;
    if (this->engine)
        delete this->engine;
    if (this->runtime)
        delete this->runtime;
    if (this->stream)
        cudaStreamDestroy(this->stream);
}
```

相比旧版，新增了 5 个 `cudaFree` / `cudaFreeHost` 调用，释放所有新增的 GPU / Pinned 缓冲区。

释放顺序依然是从内到外：CUDA buffers → TensorRT 对象 → CUDA stream。

---

## 四、GPU 预处理：`preprocessing`（重大重写）

```cpp
void Model::preprocessing(const cv::Mat &frame)
{
    int img_w = frame.cols;
    int img_h = frame.rows;

    float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
    int new_w = int(img_w * scale);
    int new_h = int(img_h * scale);

    this->rx = (float)img_w / new_w;
    this->ry = (float)img_h / new_h;
```

前半部分和旧版相同：计算 Letterbox 缩放比例和恢复系数。

---

### 上传原始图像到 GPU

```cpp
    size_t imgBytes = frame.step[0] * frame.rows;
    if (gpuInputCapacity_ < imgBytes) {
        if (gpuInputBuffer8U_) cudaFree(gpuInputBuffer8U_);
        gpuInputBuffer8U_ = nullptr;
        cudaError_t err = cudaMalloc(&gpuInputBuffer8U_, imgBytes);
        if (err != cudaSuccess) {
            gpuInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc gpuInputBuffer8U_ failed: ") + cudaGetErrorString(err));
        }
        gpuInputCapacity_ = imgBytes;
    }

    cudaMemcpyAsync(gpuInputBuffer8U_, frame.data, imgBytes, cudaMemcpyHostToDevice, this->stream);
```

步骤：
1. 计算原始图像字节数（考虑行步长 `frame.step[0]`，可能有 padding）
2. 如果 GPU staging buffer 不够大，重新分配（grow-on-demand）
3. 异步上传原始图像（BGR uint8）到 GPU

---

### 调用 CUDA 预处理 Kernel

```cpp
    launch_preprocess(
        static_cast<const uint8_t*>(gpuInputBuffer8U_),
        img_w, img_h, static_cast<int>(frame.step[0]),
        static_cast<float*>(buffers[0]),
        input_w, input_h,
        static_cast<float>(img_w) / new_w,
        static_cast<float>(img_h) / new_h,
        new_w, new_h,
        mean_, std_,
        true,   // swapRB (BGR→RGB)
        this->stream
    );
}
```

这是**最关键的变更**。旧版在 CPU 上完成所有预处理（`cv::resize` → `blobFromImage` → 归一化 → `cudaMemcpyAsync`），新版把**整个预处理流程搬到 GPU**：

```text
旧版（CPU 预处理）：
  原图(CPU) → resize(CPU) → letterbox(CPU) → blobFromImage(CPU) → H2D → buffers[0]

新版（GPU 预处理）：
  原图(CPU) → H2D → gpuInputBuffer8U_(GPU) → launch_preprocess(GPU kernel) → buffers[0]
```

GPU kernel 内部完成：
1. Bilinear resize
2. Letterbox padding（左上角对齐，灰色 padding=114）
3. BGR→RGB 通道交换
4. 归一化（`/255.0`）
5. 可选 ImageNet 标准化（`(x - mean) / std`）
6. HWC→CHW 格式转换

**所有操作在 GPU 上完成**，省掉了旧版的 CPU resize + blobFromImage + 逐像素归一化的开销。

详见 `preprocess.hpp` / `preprocess.cu` 的文档。

---

## 五、CPU 预处理：`preprocessSingle`（保留为 fallback）

```cpp
cv::Mat Model::preprocessSingle(const cv::Mat& frame, float& rx, float& ry)
{
    int img_w = frame.cols;
    int img_h = frame.rows;

    float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
    int new_w = int(img_w * scale);
    int new_h = int(img_h * scale);

    cv::Mat resized_img;
    cv::resize(frame, resized_img, cv::Size(new_w, new_h));

    cv::Mat canvas = cv::Mat::zeros(this->input_h, this->input_w, CV_8UC3);
    canvas.setTo(cv::Scalar(114, 114, 114));
    resized_img.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));

    rx = (float)img_w / new_w;
    ry = (float)img_h / new_h;

    cv::Mat blob = cv::dnn::blobFromImage(canvas, 1 / 255.0,
        cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);

    if (this->output_h == 1 || this->output_w == 1) { 
        float mean[] = {0.485, 0.456, 0.406};
        float std[] = {0.229, 0.224, 0.225};
        float* data = (float*)blob.data;
        int pixels_per_channel = this->input_w * this->input_h;
        for (int c = 0; c < 3; ++c) {
            for (int i = 0; i < pixels_per_channel; ++i) {
                data[c * pixels_per_channel + i] = (data[c * pixels_per_channel + i] - mean[c]) / std[c];
            }
        }
    }
    return blob;
}
```

这个方法保留了旧版的**纯 CPU 预处理路径**，作为 `postprocessSingle` 和批量推理 Slow fallback 的辅助函数。

输入是 `cv::Mat`，输出也是 `cv::Mat`（blob），不涉及 GPU 操作。

---

## 六、后处理重构：`postprocessing`

```cpp
void Model::postprocessing()
{
    if (this->modelType == ModelType::CLASSIFY) {
        return;
    }

    this->detectResults.clear();
    cv::Mat det_output(this->output_h, this->output_w, CV_32F, prob_);
    this->detectResults = postprocessSingle(det_output, this->rx, this->ry);
}
```

旧版直接在 `postprocessing` 中写所有解析逻辑。新版重构为：
1. `postprocessing` 只做 D2H 数据搬运 + 调用 `postprocessSingle`
2. 真正的解析逻辑在 `postprocessSingle` 中，可以被 `postprocessBatch` 复用

注意：新版用 `prob_`（Pinned Memory 指针）替代了旧版的 `prob.data()`（vector 数据指针）。

---

## 七、`postprocessSingle`（新增，后处理核心）

```cpp
std::vector<Result> Model::postprocessSingle(const cv::Mat& det_output, float rx, float ry)
{
    std::vector<Result> results;
    
    if (this->modelType == ModelType::CLASSIFY) {
        return results;
    }

    if (this->isNMS) 
    {
        // ... 解析 end-to-end NMS 输出（和旧版基本相同）
    } 
    else 
    {
        // ... 手动 NMS（和旧版基本相同）
    }
    
    return results;
}
```

把旧版 `postprocessing` 中的解析逻辑提取为独立函数，接收 `cv::Mat` 输出和 `rx`/`ry` 参数，返回结果向量。

这样 `postprocessBatch` 可以对 batch 中的每个样本调用 `postprocessSingle`，避免代码重复。

---

## 八、`postprocessBatch`（新增）

```cpp
std::vector<std::vector<Result>> Model::postprocessBatch(
    const float* outputData, int batchSize,
    const std::vector<float>& rxs, const std::vector<float>& rys)
{
    std::vector<std::vector<Result>> batchResults(batchSize);
    if (this->modelType == ModelType::CLASSIFY) {
        return batchResults;
    }
    for (int b = 0; b < batchSize; ++b) {
        const float* sampleData = outputData + b * output_h * output_w;
        cv::Mat det_output(this->output_h, this->output_w, CV_32F, const_cast<float*>(sampleData));
        batchResults[b] = postprocessSingle(det_output, rxs[b], rys[b]);
    }
    return batchResults;
}
```

对 batch 输出数据逐样本解析：
1. 用指针偏移定位每个样本的输出数据
2. 包装成 `cv::Mat`（零拷贝）
3. 调用 `postprocessSingle`

---

## 九、`Detect` 主函数（API 升级）

```cpp
bool Model::Detect(const cv::Mat &frame)
{
    try
    {
        this->detectResults.clear();

        preprocessing(frame);

        context->setTensorAddress(inputName_.c_str(), buffers[0]);
        context->setTensorAddress(outputName_.c_str(), buffers[1]);
        nvinfer1::Dims4 inputDims(1, 3, input_h, input_w);
        context->setInputShape(inputName_.c_str(), inputDims);

        if (!context->enqueueV3(this->stream)) {
            throw std::runtime_error("enqueueV3 failed in Detect");
        }

        cudaMemcpyAsync(this->prob_, this->buffers[1],
                        this->probSize_ * sizeof(float),
                        cudaMemcpyDeviceToHost, this->stream);
        cudaStreamSynchronize(this->stream);

        postprocessing();

        return !detectResults.empty();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}
```

---

### TensorRT 10.x API 变化

旧版：
```cpp
context->executeV2(buffers);
```

新版：
```cpp
context->setTensorAddress(inputName_.c_str(), buffers[0]);
context->setTensorAddress(outputName_.c_str(), buffers[1]);
nvinfer1::Dims4 inputDims(1, 3, input_h, input_w);
context->setInputShape(inputName_.c_str(), inputDims);
context->enqueueV3(this->stream);
```

#### 为什么升级？

TensorRT 10.x 引入了新的 `enqueueV3` API，相比 `executeV2`：

1. **支持动态 shape**：`setInputShape` 可以在运行时设置输入维度（对批量推理至关重要）
2. **显式 tensor 地址绑定**：`setTensorAddress` 按名称设置输入输出 buffer，比按索引的 `buffers[]` 数组更清晰
3. **异步执行**：`enqueueV3` 提交到 stream 后立即返回，CPU 可以继续工作

---

### 异常处理（增强）

旧版：`executeV2` 无返回值检查
新版：`enqueueV3` 返回 `bool`，失败时 `throw std::runtime_error`

---

## 十、`predictClass`（API 升级）

```cpp
int Model::predictClass(const cv::Mat &roi) {
    preprocessing(roi);
    context->setTensorAddress(inputName_.c_str(), buffers[0]);
    context->setTensorAddress(outputName_.c_str(), buffers[1]);
    nvinfer1::Dims4 inputDims(1, 3, input_h, input_w);
    context->setInputShape(inputName_.c_str(), inputDims);

    if (!context->enqueueV3(this->stream)) {
        throw std::runtime_error("enqueueV3 failed in predictClass");
    }

    cudaMemcpyAsync(this->prob_, this->buffers[1],
                    this->probSize_ * sizeof(float),
                    cudaMemcpyDeviceToHost, this->stream);
    cudaStreamSynchronize(this->stream);
    float* data = this->prob_;
    return std::max_element(data, data + probSize_) - data;
}
```

和旧版结构相同，但：
* `executeV2` → `enqueueV3` + `setTensorAddress` + `setInputShape`
* `prob.data()` → `prob_`（Pinned Memory）

---

## 十一、批量推理实现（新增）

### `predictClassBatchSlow`

```cpp
std::vector<int> Model::predictClassBatchSlow(const std::vector<cv::Mat>& rois) {
    std::vector<int> results;
    results.reserve(rois.size());
    for (const auto& roi : rois) {
        results.push_back(predictClass(roi));
    }
    return results;
}
```

逐帧循环调用 `predictClass`。简单但低效：N 个 ROI = N 次 GPU kernel launch + N 次同步。

---

### `predictClassBatch`（高效版）

核心流程：

```text
1. 检查 engine 是否支持动态 batch（d[0] == -1）
2. 不支持 → 退化到 predictClassBatchSlow
3. 计算总 buffer 大小，ensureBatchBuffers
4. 逐个 ROI：
   a. 上传原始图像到 GPU staging buffer
   b. 调用 launch_preprocess 预处理到 batchInputBuffer_ 的对应偏移
5. setInputShape(N, 3, input_h, input_w) 设置 batch size
6. enqueueV3 一次推理
7. D2H 拷贝所有输出
8. 逐样本取 argmax
9. 重置 batch size 为 1
```

关键点：
- **动态 batch 检测**：`engine->getTensorShape(inputName_.c_str())` 检查 `d[0] == -1`
- **GPU 预处理**：每个 ROI 的预处理都在 GPU 上完成，避免 CPU resize 的开销
- **Fallback 安全**：如果 `enqueueV3` 失败，自动重置 batch size 并退化到 Slow 路径
- **Batch size 重置**：推理完成后立即 `setInputShape(1, 3, h, w)`，确保后续单帧推理不受影响

---

### `DetectBatchSlow` / `DetectBatch`

结构和分类版本类似，但后处理使用 `postprocessBatch` 解析检测框：

```cpp
std::vector<std::vector<Result>> Model::DetectBatch(const std::vector<cv::Mat>& rois) {
    // ... 类似 predictClassBatch，但：
    // 1. 记录每个 ROI 的 rxs/rys
    // 2. D2H 后调用 postprocessBatch(hOutput_, N, rxs, rys)
}
```

---

## 十二、`ensureBatchBuffers`（新增）

```cpp
void Model::ensureBatchBuffers(size_t inputBytes, size_t outputBytes) {
    if (batchInputCapacity_ < inputBytes) {
        if (batchInputBuffer_) cudaFree(batchInputBuffer_);
        batchInputBuffer_ = nullptr;
        cudaError_t err = cudaMalloc(&batchInputBuffer_, inputBytes);
        if (err != cudaSuccess) {
            batchInputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc batchInputBuffer_ failed: ") + cudaGetErrorString(err));
        }
        batchInputCapacity_ = inputBytes;
    }
    if (batchOutputCapacity_ < outputBytes) {
        if (batchOutputBuffer_) cudaFree(batchOutputBuffer_);
        batchOutputBuffer_ = nullptr;
        cudaError_t err = cudaMalloc(&batchOutputBuffer_, outputBytes);
        if (err != cudaSuccess) {
            batchOutputCapacity_ = 0;
            throw std::runtime_error(std::string("cudaMalloc batchOutputBuffer_ failed: ") + cudaGetErrorString(err));
        }
        batchOutputCapacity_ = outputBytes;
    }
}
```

**Grow-on-demand** 模式的 GPU buffer 管理。只在需要更大 buffer 时才重新分配，减少 `cudaMalloc` 调用次数。

---

# 第三部分：从 Model 层学到的设计要点

## 1. GPU 预处理流水线

```text
原图(CPU) → H2D → GPU kernel(resize+letterbox+normalize+HWC→CHW) → buffers[0]
```

把 CPU 密集型的预处理搬到 GPU，是推理性能优化的**第一步**。CPU 只负责上传原始字节，所有像素级操作都在 GPU 上完成。

## 2. TensorRT 10.x 的 `enqueueV3` API

```cpp
context->setTensorAddress(name, buffer);
context->setInputShape(name, dims);
context->enqueueV3(stream);
```

相比旧的 `executeV2`，新 API 支持动态 batch、显式 tensor 绑定、异步执行。

## 3. Pinned Memory 加速 D2H 拷贝

```cpp
cudaMallocHost(&prob_, size);  // Pinned memory
cudaMemcpyAsync(prob_, gpu_buf, size, D2H, stream);  // 真正异步
```

Pinned Memory 确保 `cudaMemcpyAsync` 不退化成同步拷贝。

## 4. 批量推理减少 GPU kernel launch 开销

N 个 ROI 的分类：
* 逐帧：N 次 launch + N 次 sync
* 批量：1 次 launch + 1 次 sync

在分类模型推理密集时，收益可达 30%+。

## 5. Grow-on-demand Buffer 管理

```cpp
if (capacity < needed) {
    cudaFree(buffer);
    cudaMalloc(&buffer, needed);
    capacity = needed;
}
```

避免一开始就分配最大 buffer（浪费显存），也避免每帧重新分配（开销大）。

## 6. 从 `assert` 到 `throw` 的错误处理演进

```cpp
// 旧版：Release 编译时 assert 被消除
assert(this->runtime != nullptr);

// 新版：始终生效
if (!this->runtime) {
    throw std::runtime_error("...");
}
```

在依赖外部资源（GPU、文件、CUDA 驱动）的代码中，`assert` 是不够的。

## 7. CPU fallback 路径保留

`preprocessSingle`（CPU 预处理）和 `Slow` 后缀的批量方法被保留，作为 GPU 初始化失败或不支持动态 batch 时的 fallback。这种**双路径设计**提高了代码的健壮性。
