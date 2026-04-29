# `model.hpp` + `model.cpp` 逐行讲解

这两份文件是项目的**推理引擎核心**，负责 TensorRT 模型的加载、前处理、推理、后处理。整个检测系统的速度和精度，最终都取决于这一层的实现质量。

---

# 第一部分：`model.hpp`

## 一、头文件保护

```cpp
#ifndef __MODEL_HPP__
#define __MODEL_HPP__
```

经典的 C/C++ **头文件包含保护（Include Guard）**。作用是防止同一个头文件被重复包含导致编译错误。

如果改用 `#pragma once` 效果一样，但 `#ifndef` 是标准 C++ 兼容所有编译器的方式。

---

## 二、引入的头文件

```cpp
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
```

---

### `<iostream>` / `<vector>`

标准 C++ 库：输入输出流、动态数组容器。

---

### `<opencv2/opencv.hpp>`

OpenCV 全功能头文件。这里主要用到：

* `cv::Mat`：图像矩阵
* `cv::Rect`：矩形框
* `cv::Point2f`：二维浮点坐标
* `cv::resize`、`cv::dnn::blobFromImage`、`cv::dnn::NMSBoxes` 等

---

### `<opencv2/dnn.hpp>`

OpenCV 的深度学习模块。虽然推理用的是 TensorRT，但预处理和后处理借用了 OpenCV DNN 的一些工具函数：

* `cv::dnn::blobFromImage`：图像转网络输入 blob
* `cv::dnn::NMSBoxes`：非极大值抑制

---

### `<NvInfer.h>`

**NVIDIA TensorRT 的核心头文件**。它定义了：

* `nvinfer1::IRuntime`：运行时，负责反序列化引擎
* `nvinfer1::ICudaEngine`：推理引擎，包含网络结构和优化后的计算图
* `nvinfer1::IExecutionContext`：执行上下文，负责实际推理
* `nvinfer1::ILogger`：日志接口
* `nvinfer1::Dims`、`nvinfer1::TensorIOMode` 等数据结构

没有它，就没法和 TensorRT 交互。

---

### `<cuda_runtime_api.h>`

CUDA 运行时 API。这里用到：

* `cudaMalloc`：在 GPU 上分配显存
* `cudaFree`：释放 GPU 显存
* `cudaMemcpyAsync`：异步内存拷贝（Host ↔ Device）
* `cudaStreamCreate` / `cudaStreamDestroy`：CUDA 流管理
* `cudaStreamSynchronize`：流同步

TensorRT 的推理完全在 GPU 上进行，所以必须用 CUDA API 管理显存和数据传输。

---

## 三、`Result` 结构体

```cpp
struct Result
{
    int idx = 0;              // 类别 ID
    float confidence = 0.0f;  // 置信度
    cv::Rect box;             // 检测框（像素坐标）
    int armorColor = 0;       // 装甲颜色 / 队伍 ID
    cv::Rect car_box{};       // 车辆整体框
    cv::Point2f worldPoint{}; // 世界坐标（像素级中间表示）
    float fps = 0.0f;         // 帧率（传递用）
    bool isDead = false;      // 死亡装甲板标志
};
```

这是整个项目**最核心的数据结构**。从检测到可视化到 ROS2 消息，几乎所有模块都在传递 `Result` 或它的变体。

---

### 字段设计解读

| 字段 | 类型 | 含义 |
|------|------|------|
| `idx` | `int` | 类别索引。0=CAR, 1=ARMOR, 2=R1, ..., 6=S |
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
2. 下游节点（如地图绘制）无法准确判断一个 `armorColor == 0` 的目标是"未识别颜色的存活装甲板"还是"死亡装甲板"

引入独立的 `isDead` 标志后：
- `isDead == true` → 死亡装甲板，不进入分类，在地图上显示为黑色 `"dead"`
- `isDead == false` → 存活装甲板，正常进入分类，在地图上按 `armorColor` 显示红/蓝

---

### 为什么用一个结构体贯穿全流程？

这体现了 C++ **聚合数据（Aggregate Data）** 的设计思想：

> 把同一目标的所有相关信息打包在一起，避免函数参数列表过长，也避免多个独立数组之间的索引同步问题。

如果没有 `Result`，`pipeline` 可能需要返回七八个独立的 `vector`，维护它们之间的对应关系会非常痛苦。

---

## 四、`Model` 类声明

```cpp
class Model
{
```

`Model` 是一个**TensorRT 推理封装类**。每个 `Model` 实例对应一个加载到 GPU 上的 TensorRT 引擎。

在你的项目里，同时存在三个 `Model` 实例：

1. `detectModel_`：第一阶段目标检测（检测车辆）
2. `armorDetector_`：第二阶段装甲板检测（在车辆 ROI 内检测装甲板）
3. `classifyModel_`：第三阶段分类（识别装甲板属于哪个机器人）

---

## 五、私有成员变量

```cpp
private:
    int inputSize;        // 模型输入尺寸（正方形边长）
    float scoreThreshold; // 置信度阈值
    float nmsThreshold;   // NMS IoU 阈值
    bool isNMS;           // 是否启用 NMS
```

---

### `inputSize`

模型输入图像的边长。比如 YOLO 通常用 640×640，分类模型可能用 224×224。

实际输入图像可能不是正方形，所以预处理时会做**等比例缩放 + 灰边填充（Letterbox）**。

---

### `scoreThreshold`

置信度过滤阈值。模型输出成千上万个候选框，只有 confidence > scoreThreshold 的才保留。

这是**后处理的第一道关卡**，把明显是背景的低分预测剪掉。

---

### `nmsThreshold`

NMS（Non-Maximum Suppression，非极大值抑制）的 IoU 阈值。

NMS 是目标检测后处理的核心算法：对于同一物体的多个重叠框，只保留置信度最高的那个，抑制（删除）和它重叠度（IoU）过高的其他框。

`nmsThreshold` 通常设 0.45~0.65。越大，允许重叠的框越多；越小，越严格。

---

### `isNMS`

控制后处理流程的分支：

* `true`：模型已经做了内置 NMS（如 YOLOv8 的 end-to-end 输出）
* `false`：模型输出原始 anchor 预测，需要手动做 NMS

---

## 六、TensorRT 核心对象

```cpp
    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
```

这是 TensorRT **推理三件套**，三者有严格的依赖关系和生命周期：

```text
IRuntime ──反序列化──→ ICudaEngine ──创建──→ IExecutionContext
```

---

### `IRuntime`

TensorRT 运行时对象。它知道如何：

* 从序列化文件中读取引擎定义
* 在当前 GPU 上重建优化后的计算图

一个程序通常只需要一个 `IRuntime`，但你的代码里每个 `Model` 实例都创建了自己的 `runtime`。

---

### `ICudaEngine`

推理引擎，代表一个**优化后的神经网络**。它包含：

* 网络结构（层、连接关系）
* TensorRT 做的平台相关优化（算子融合、精度校准、内核自动调优等）
* 输入/输出 tensor 的形状信息

`ICudaEngine` 是**线程安全的只读对象**，可以被多个 `IExecutionContext` 共享。

---

### `IExecutionContext`

执行上下文，负责**实际执行推理**。它包含：

* 推理过程中需要的临时显存
* 执行状态

每个 `IExecutionContext` 只能同时执行一个推理请求。如果要并行推理多个 batch，需要创建多个 context。

在你的代码中，每个 `Model` 只有一个 context，所以是**单线程串行推理**。

---

## 七、CUDA 资源

```cpp
    cudaStream_t stream = nullptr;
    void *buffers[2] = {nullptr, nullptr};
    std::vector<float> prob;
```

---

### `stream`

CUDA 流（Stream）。CUDA 操作（内存拷贝、内核执行）可以提交到流中异步执行。

使用流的好处：

* **CPU 和 GPU 可以并行工作**：CPU 准备下一帧数据时，GPU 在后台推理上一帧
* **流水线化**：`H2D` 拷贝 → `推理` → `D2H` 拷贝 可以部分重叠

---

### `buffers[2]`

GPU 显存指针数组。`buffers[0]` 指向输入 buffer，`buffers[1]` 指向输出 buffer。

TensorRT 的 `executeV2` 要求传入一个 `void*[]` 数组，按绑定索引顺序提供输入和输出显存地址。

---

### `prob`

CPU 内存中的输出缓冲区。推理完成后，GPU 输出通过 `cudaMemcpyAsync` 拷贝到 `prob` 中，供 CPU 后处理读取。

---

## 八、输入输出维度

```cpp
    int input_h, input_w;
    int output_h, output_w;
    float rx, ry;
```

---

### `input_h` / `input_w`

模型实际输入 tensor 的高度和宽度。从 engine 的 tensor shape 中读取，不一定等于 `inputSize`（虽然通常一致）。

---

### `output_h` / `output_w`

模型输出 tensor 的维度。对于检测模型，输出通常是 `(num_predictions, num_properties)` 或转置形式。

---

### `rx` / `ry`

**缩放恢复系数**。预处理时图像被缩放到模型输入尺寸，后处理时需要把检测框坐标**映射回原图尺寸**。

```text
原图坐标 = 模型输出坐标 × rx (或 ry)
```

这是目标检测中**Letterbox 预处理**的标准配套计算。

---

## 九、内部方法

```cpp
    cv::Mat resizeFrame;
    void preprocessing(const cv::Mat &frame);
    void postprocessing();
```

---

### `preprocessing`

图像预处理：缩放、填充、归一化、格式转换（HWC → CHW），最后把数据拷到 GPU 输入 buffer。

---

### `postprocessing`

结果后处理：从 GPU 输出 buffer 读取原始预测，经过置信度过滤、NMS、坐标映射，生成 `detectResults`。

---

## 十、公共接口

```cpp
public:
    enum class ModelType
    {
        DETECT,    // 检测模型（输出检测框）
        CLASSIFY,  // 分类模型（输出类别概率）
        UNKNOWN
    };
```

---

### `ModelType`

区分模型的用途，影响后处理逻辑：

* `DETECT`：后处理走检测流程（解析 box + confidence + class）
* `CLASSIFY`：后处理直接取最大概率类别，不需要解析坐标

---

### 成员变量

```cpp
    std::vector<Result> detectResults;  // 检测结果缓存
    ModelType modelType;                // 当前模型类型
```

`detectResults` 是每次 `Detect()` 调用后存放结果的地方。外部代码（如 `pipeline`）从这里读取检测结果。

---

### 构造函数

```cpp
    Model(const std::string modelPath, const int &inputSize,
          const float &scoreThreshold, const float &nmsThreshold,
          const bool isNMS = true,
          const ModelType modelType = ModelType::DETECT);
```

参数：

| 参数 | 含义 |
|------|------|
| `modelPath` | TensorRT engine 文件路径（`.engine`） |
| `inputSize` | 期望的输入尺寸 |
| `scoreThreshold` | 置信度阈值 |
| `nmsThreshold` | NMS IoU 阈值 |
| `isNMS` | 是否启用 NMS 后处理 |
| `modelType` | 模型类型 |

---

### 析构函数

```cpp
    ~Model();
```

负责按正确顺序释放 TensorRT 和 CUDA 资源：

```text
context → engine → runtime
CUDA buffers → stream
```

顺序很重要：必须先释放依赖者，再释放被依赖者。

---

### `predictClass`

```cpp
    int predictClass(const cv::Mat &roi);
```

分类专用接口。输入一个 ROI 图像，返回预测的类别 ID。

`classifyModel_` 在 `pipeline` 中就是调用这个方法。

---

### `roi`

```cpp
    cv::Rect roi;
```

公共成员变量。在检测模型中，它可能被用来传递某种 ROI 信息。

---

### `Detect`

```cpp
    bool Detect(const cv::Mat &frame);
```

检测主接口。输入一帧图像，内部执行：

```text
preprocessing → executeV2(推理) → postprocessing
```

返回 `true` 表示检测到了目标（`detectResults` 非空）。

---

## 十一、头文件保护结束

```cpp
#endif
```

对应开头的 `#ifndef __MODEL_HPP__`。

---

---

# 第二部分：`model.cpp`

## 一、Logger 类

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

### 为什么需要 Logger？

TensorRT 在反序列化引擎、构建网络、执行推理时，会产生大量内部日志（层融合信息、优化决策、警告等）。

`nvinfer1::ILogger` 是一个**抽象接口**，TensorRT 要求用户提供一个实现，来决定：

* 什么级别的日志需要输出
* 输出到哪里（控制台、文件、日志系统）

---

### `noexcept override`

* `override`：明确表示这是重写父类虚函数，编译器会检查签名是否匹配
* `noexcept`：承诺这个函数不抛异常。TensorRT 的回调要求 `noexcept`，因为它可能在异常不安全的环境中调用

---

### 日志级别过滤

```cpp
if (severity <= Severity::kINFO)
```

TensorRT 的日志级别从低到高：

```text
kVERBOSE < kINFO < kWARNING < kERROR < kINTERNAL_ERROR
```

这行代码的意思是：只输出 `kINFO` 及以上级别的日志（即 INFO、WARNING、ERROR）。`kVERBOSE` 被过滤掉，避免刷屏。

---

### `gLogger`

全局单例对象。所有 `Model` 实例共用同一个 logger。

---

## 二、构造函数

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

先把参数保存到成员变量。这里用 `this->` 显式区分成员变量和参数，虽然不加也可以，但增加了可读性。

---

## 三、读取 Engine 文件

```cpp
    std::ifstream engineFile(modelPath, std::ios::binary);
    std::vector<char> engineData;
    int fsize = 0;

    if (engineFile.good())
    {
        engineFile.seekg(0, engineFile.end);
        fsize = engineFile.tellg();
        engineFile.seekg(0, engineFile.beg);
        engineData.resize(fsize);
        engineFile.read(engineData.data(), fsize);
        engineFile.close();
    }
```

---

### 为什么用 `std::ios::binary`？

TensorRT 的 engine 文件是**二进制格式**，包含序列化的网络结构、权重、优化信息等。必须用二进制模式读取，否则换行符转换会破坏数据。

---

### 文件读取流程

1. `seekg(0, end)`：把读指针移到文件末尾
2. `tellg()`：获取当前位置，即文件大小
3. `seekg(0, beg)`：把读指针移回开头
4. `resize(fsize)`：给 `engineData` 分配刚好够的内存
5. `read(data, fsize)`：一次性读取全部内容

这是 C++ 读取二进制文件的**标准高效做法**，避免了多次内存重分配。

---

## 四、反序列化引擎

```cpp
    this->runtime = nvinfer1::createInferRuntime(gLogger);
    assert(this->runtime != nullptr);

    this->engine = this->runtime->deserializeCudaEngine(engineData.data(), fsize);
    assert(this->engine != nullptr);

    this->context = this->engine->createExecutionContext();
    assert(this->context != nullptr);
```

---

### `createInferRuntime`

创建 TensorRT 运行时实例。它需要和 `nvinfer1::ILogger` 一起工作，所以传入 `gLogger`。

---

### `deserializeCudaEngine`

**反序列化**。把文件里的字节流还原成 GPU 上的可执行引擎。

这个过程会：

* 解析网络结构
* 在当前 GPU 架构上重新编译优化内核
* 分配必要的资源

反序列化是**耗时的操作**（可能几百毫秒到几秒），所以只在构造函数里做一次，后续推理反复复用。

---

### `assert`

调试断言。如果创建失败，程序直接终止。

在生产环境中，通常应该把 `assert` 换成更友好的错误处理（抛异常或返回错误码）。

---

## 五、解析输入输出 Tensor 名称

```cpp
    std::string inputName, outputName;
    int nbTensors = this->engine->getNbIOTensors();
    for (int i = 0; i < nbTensors; ++i) {
        const char* name = this->engine->getIOTensorName(i);
        if (this->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kINPUT) {
            inputName = name;
        } else if (this->engine->getTensorIOMode(name) == nvinfer1::TensorIOMode::kOUTPUT) {
            outputName = name;
        }
    }
```

TensorRT 10.x 使用 **I/O Tensor** 机制管理输入输出。每个 tensor 有名字和模式（INPUT / OUTPUT）。

这段代码遍历所有 I/O tensor，自动识别输入和输出名字。这意味着：

> **你的代码不硬编码 tensor 名字（如 "images"、"output0"），而是自动推断。**

这提高了代码的通用性，换不同的 ONNX 模型导出引擎后，不需要改代码。

---

## 六、获取输入输出维度

```cpp
    nvinfer1::Dims inputDims = this->engine->getTensorShape(inputName.c_str());
    nvinfer1::Dims outputDims = this->engine->getTensorShape(outputName.c_str());

    this->input_h = inputDims.d[2];
    this->input_w = inputDims.d[3];

    if (outputDims.nbDims == 2) {
        this->output_h = outputDims.d[0];
        this->output_w = outputDims.d[1];
    } else {
        this->output_h = outputDims.d[1];
        this->output_w = outputDims.d[2];
    }
```

---

### `nvinfer1::Dims`

TensorRT 的多维数组描述结构。`d[0]`, `d[1]`, `d[2]`, `d[3]` 分别对应 N、C、H、W（batch、通道、高、宽）。

对于图像输入，通常 shape 是 `(1, 3, 640, 640)`，所以：

* `d[0] = 1`（batch size）
* `d[1] = 3`（RGB 通道）
* `d[2] = 640`（高）
* `d[3] = 640`（宽）

代码取 `d[2]` 和 `d[3]` 作为 `input_h` 和 `input_w`。

---

### 输出维度判断

不同模型的输出格式不同：

* YOLOv8 end-to-end：`(1, 84, 8400)` 或 `(1, 8400, 84)`
* 传统 YOLO：`(1, 25200, 85)`
* 分类模型：`(1, num_classes)`

代码通过 `nbDims == 2` 判断是 2D 输出还是 3D 输出，然后取对应维度。

---

## 七、GPU 显存分配

```cpp
    cudaMalloc(&(this->buffers[0]), this->input_h * this->input_w * 3 * sizeof(float));
    cudaMalloc(&(this->buffers[1]), this->output_h * this->output_w * sizeof(float));
    this->prob.resize(this->output_h * this->output_w);
    cudaStreamCreate(&(this->stream));
```

---

### `cudaMalloc`

在 GPU 显存上分配空间。

* `buffers[0]`：输入 buffer 大小 = `H × W × 3 × sizeof(float)`（RGB 三通道浮点图）
* `buffers[1]`：输出 buffer 大小 = `output_h × output_w × sizeof(float)`

注意这里假设输入数据类型是 `float32`。如果模型量化成 INT8，需要调整。

---

### `prob.resize`

在 CPU 内存上分配同样大小的输出缓冲区。推理完成后，GPU 输出会拷贝到这里。

---

### `cudaStreamCreate`

创建 CUDA 流。后续的 `cudaMemcpyAsync` 和 `executeV2` 都会提交到这个流中异步执行。

---

## 八、析构函数

```cpp
Model::~Model()
{
    for (auto &buffer : this->buffers)
    {
        if (buffer)
            cudaFree(buffer);
    }
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

---

### 释放顺序

```text
1. CUDA buffers（显存）
2. IExecutionContext
3. ICudaEngine
4. IRuntime
5. CUDA stream
```

这是**从内到外、从依赖到被依赖**的顺序。如果先释放 `runtime`，再通过 `engine` 或 `context` 访问就会崩溃。

---

### `if (buffer)` 检查

防止重复释放或释放空指针。虽然 `cudaFree(nullptr)` 是安全的（CUDA 规定它为 no-op），但显式检查是好的习惯。

---

## 九、预处理：`preprocessing`

```cpp
void Model::preprocessing(const cv::Mat &frame)
{
    int img_w = frame.cols;
    int img_h = frame.rows;

    float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
    int new_w = int(img_w * scale);
    int new_h = int(img_h * scale);
```

---

### 计算缩放比例

```cpp
float scale = std::min(input_w / img_w, input_h / img_h);
```

这是 **Letterbox 缩放**的核心：

> 保持原始图像的宽高比，等比例缩放到能放进模型输入框的最大尺寸。

例如原图是 1920×1080，模型输入是 640×640：

* `scale_x = 640 / 1920 = 0.333`
* `scale_y = 640 / 1080 = 0.593`
* `scale = min(0.333, 0.593) = 0.333`

所以新尺寸是 `640 × 360`，保持原比例，且能放进 640×640 的方框。

---

### OpenCV 缩放

```cpp
    cv::Mat resized_img;
    cv::resize(frame, resized_img, cv::Size(new_w, new_h));
```

用双线性插值把图像缩放到计算出的新尺寸。

---

### 灰边填充（Letterbox Padding）

```cpp
    cv::Mat canvas = cv::Mat::zeros(this->input_h, this->input_w, CV_8UC3);
    canvas.setTo(cv::Scalar(114, 114, 114));
    resized_img.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));
```

---

### `cv::Mat::zeros`

创建一个全黑的图像，尺寸为模型输入大小（如 640×640）。

---

### `setTo(114, 114, 114)`

把整个画布填充成灰色（RGB=114）。

为什么是 114？这是 YOLO 系列训练时的默认 padding 值。用灰色而不是黑色，是因为：

* 黑色可能在某些场景下被模型误识别为"暗区"
* 灰色更接近自然图像的均值，对模型干扰最小

---

### `copyTo`

把缩放后的图像贴在画布的左上角 `(0, 0)`。

因为图像是等比例缩放的，且 `new_w <= input_w`、`new_h <= input_h`，所以一定能放下。右边和下边会留下灰色填充区。

---

### 记录缩放系数

```cpp
    this->rx = (float)img_w / new_w;
    this->ry = (float)img_h / new_h;
```

保存**原图尺寸 / 缩放后尺寸**的比例。后处理时，模型输出的坐标是在 `new_w × new_h` 尺度上的，需要乘以 `rx`/`ry` 才能映射回原图。

---

### Blob 转换

```cpp
    cv::Mat canvas_non_const = canvas.clone();
    cv::Mat blob = cv::dnn::blobFromImage(canvas_non_const, 1 / 255.0,
                                          cv::Size(this->input_w, this->input_h),
                                          cv::Scalar(0, 0, 0), true, false);
```

---

### `cv::dnn::blobFromImage`

OpenCV DNN 模块提供的预处理函数，一步完成多项转换：

1. **缩放像素值**：`1/255.0` 把 `[0, 255]` 映射到 `[0, 1]`
2. **调整尺寸**：虽然已经是 `input_w × input_h`，但再确认一次
3. **减去均值**：`cv::Scalar(0, 0, 0)` 表示不减均值（后续单独处理）
4. **交换通道**：`true` 表示 `swapRB`，把 BGR 转成 RGB
5. **裁剪**：`false` 表示不裁剪

输出 `blob` 的格式是 **NCHW**：`(1, 3, H, W)`，这是大多数深度学习框架的标准输入格式。

---

### 分类模型的特殊归一化

```cpp
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
```

---

### 为什么有这段特殊处理？

当 `output_h == 1 || output_w == 1` 时，说明输出是 1D 向量，这通常是**分类模型**的特征。

分类模型（如 ResNet、RepVGG）在 ImageNet 上预训练时，使用的预处理是：

```text
x = (x / 255.0 - mean) / std
```

其中 `mean = [0.485, 0.456, 0.406]`，`std = [0.229, 0.224, 0.225]`。

检测模型（YOLO）通常不需要这个额外的归一化，所以用 `output` 维度来判断模型类型，做条件处理。

---

### 拷贝到 GPU

```cpp
    cudaMemcpyAsync(buffers[0], blob.ptr<float>(),
                    3 * this->input_h * this->input_w * sizeof(float),
                    cudaMemcpyHostToDevice, this->stream);
```

把 CPU 内存中的 `blob` 异步拷贝到 GPU 显存 `buffers[0]`。

* `cudaMemcpyHostToDevice`：方向是 CPU → GPU
* `this->stream`：提交到 CUDA 流中异步执行

"异步"意味着 CPU 不会在这里阻塞等待拷贝完成，而是继续执行后续代码。真正的同步点在 `cudaStreamSynchronize`。

---

## 十、后处理：`postprocessing`

```cpp
void Model::postprocessing()
{
    if (this->modelType == ModelType::CLASSIFY) {
        return;
    }

    this->detectResults.clear();
```

如果是分类模型，后处理直接返回。分类的结果在 `predictClass` 中单独处理。

`detectResults.clear()` 清空上一轮的结果，准备填充新结果。

---

### 构造输出矩阵视图

```cpp
    cv::Mat det_output(this->output_h, this->output_w, CV_32F, (float *)prob.data());
```

把 CPU 缓冲区 `prob` 包装成 `cv::Mat`，方便用 OpenCV 的矩阵操作来解析。

* `CV_32F`：32 位浮点
* `(float*)prob.data()`：指向 `std::vector<float>` 的底层数据指针

这是 C++ **零拷贝（Zero Copy）**技巧：`cv::Mat` 不复制数据，只是给已有内存加一层矩阵视图。

---

## 十一、后处理分支一：`isNMS == true`

```cpp
    if (this->isNMS)
    {
        bool is_transposed = (this->output_h < this->output_w);
        int num_boxes = is_transposed ? this->output_w : this->output_h;
```

---

### 判断输出格式

YOLO 类模型的输出有两种常见格式：

* **格式 A**：`(num_boxes, num_properties)` → `output_h > output_w` 不成立
* **格式 B**：`(num_properties, num_boxes)` → `output_h < output_w`

代码通过 `output_h < output_w` 判断是否是转置格式，然后确定 `num_boxes` 应该取哪个维度。

---

### 遍历所有预测框

```cpp
        for (int i = 0; i < num_boxes; ++i)
        {
            float x1, y1, x2, y2, score;
            int class_id;

            if (!is_transposed) {
                score = det_output.at<float>(i, 4);
                if (score <= this->scoreThreshold) continue;
                x1 = det_output.at<float>(i, 0);
                y1 = det_output.at<float>(i, 1);
                x2 = det_output.at<float>(i, 2);
                x2 = det_output.at<float>(i, 3);
                class_id = static_cast<int>(det_output.at<float>(i, 5));
            } else {
                score = det_output.at<float>(4, i);
                ...
            }
```

---

### 预测属性解析

每个预测框包含 6 个基本属性（假设是 end-to-end NMS 输出）：

| 索引 | 含义 |
|------|------|
| 0 | x1（左上角 x） |
| 1 | y1（左上角 y） |
| 2 | x2（右下角 x） |
| 3 | y2（右下角 y） |
| 4 | confidence（置信度） |
| 5 | class_id（类别 ID） |

根据 `is_transposed` 选择行列索引方式。

---

### 置信度过滤

```cpp
if (score <= this->scoreThreshold) continue;
```

低于阈值的预测直接跳过。这是**后处理的第一道剪枝**，通常能过滤掉 90% 以上的候选框。

---

### 坐标映射回原图

```cpp
            cv::Rect box;
            box.x = static_cast<int>(x1 * this->rx);
            box.y = static_cast<int>(y1 * this->ry);
            box.width = static_cast<int>((x2 - x1) * this->rx);
            box.height = static_cast<int>((y2 - y1) * this->ry);

            this->detectResults.emplace_back(Result{class_id, score, box});
```

模型输出的是在 Letterbox 缩放后图像上的坐标，需要乘以 `rx`/`ry` 映射回原图尺寸。

`emplace_back` 是 C++11 引入的高效插入方式，直接在 `vector` 末尾构造对象，避免一次拷贝。

---

## 十二、后处理分支二：`isNMS == false`（手动 NMS）

```cpp
    else
    {
        std::vector<cv::Rect> boxes;
        std::vector<int> classIds;
        std::vector<float> confidences;
```

当模型输出原始 anchor 预测（没做内置 NMS）时，需要手动收集所有候选框，然后做 NMS。

---

### 解析 Anchor 输出

```cpp
        bool is_transposed = (this->output_h > this->output_w);
        int num_anchors = is_transposed ? this->output_h : this->output_w;
        int num_properties = is_transposed ? this->output_w : this->output_h;
```

这里判断逻辑和分支一相反：`output_h > output_w` 表示 `(num_anchors, num_properties)` 格式。

---

### 解析每个 Anchor

```cpp
        for (int idx = 0; idx < num_anchors; ++idx)
        {
            float cx, cy, ow, oh, max_score = 0.0f;
            int class_id = 0;

            if (!is_transposed) {
                cx = det_output.at<float>(0, idx);
                cy = det_output.at<float>(1, idx);
                ow = det_output.at<float>(2, idx);
                oh = det_output.at<float>(3, idx);
```

这里的格式是 **YOLO 格式的中心坐标 + 宽高**：

| 索引 | 含义 |
|------|------|
| 0 | cx（中心 x） |
| 1 | cy（中心 y） |
| 2 | ow（框宽） |
| 3 | oh（框高） |
| 4~ | 各类别分数 或 置信度+类别分数 |

---

### 分类分数解析

```cpp
                if (num_properties == 5) {
                    max_score = det_output.at<float>(4, idx);
                    class_id = 0;
                } else {
                    cv::Mat scores = det_output.col(idx).rowRange(4, num_properties);
                    cv::Point class_id_point;
                    double score_double;
                    cv::minMaxLoc(scores, nullptr, &score_double, nullptr, &class_id_point);
                    max_score = (float)score_double;
                    class_id = class_id_point.y;
                }
```

---

#### 情况一：`num_properties == 5`

模型只输出 5 个数：`cx, cy, ow, oh, confidence`。这是**单类别检测**（如只检测车辆，不分类别）。

`class_id` 固定为 0。

---

#### 情况二：`num_properties > 5`

模型输出多类别分数。从索引 4 开始到末尾，是所有类别的分数向量。

`cv::minMaxLoc` 找出分数向量中的最大值和对应索引：

* `score_double`：最高类别分数
* `class_id_point.y`：最高分数的类别索引

这是 YOLO 经典的多类别检测后处理方式。

---

### 坐标转换和收集

```cpp
            if (max_score > this->scoreThreshold) {
                cv::Rect box;
                box.x = static_cast<int>((cx - 0.5 * ow) * this->rx);
                box.y = static_cast<int>((cy - 0.5 * oh) * this->ry);
                box.width = static_cast<int>(ow * this->rx);
                box.height = static_cast<int>(oh * this->ry);

                boxes.push_back(box);
                classIds.push_back(class_id);
                confidences.push_back(max_score);
            }
```

把中心坐标 + 宽高 转成左上角坐标 + 宽高（`cv::Rect` 的格式），同时映射回原图尺寸。

通过置信度过滤后，收集到三个平行数组中：

* `boxes`：所有候选框
* `classIds`：对应类别
* `confidences`：对应置信度

---

### 非极大值抑制（NMS）

```cpp
        std::vector<int> indexes;
        cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, indexes);
        for (int idx : indexes) {
            this->detectResults.emplace_back(Result{classIds[idx], confidences[idx], boxes[idx]});
        }
    }
}
```

---

#### `cv::dnn::NMSBoxes`

OpenCV 提供的 NMS 实现。输入：

* `boxes`：所有候选框
* `confidences`：对应置信度
* `scoreThreshold`：再次过滤低分框（双重保险）
* `nmsThreshold`：IoU 阈值，超过这个值的重叠框会被抑制

输出 `indexes`：保留下来的框的索引。

---

#### NMS 算法原理（简述）

1. 把所有框按置信度从高到低排序
2. 取置信度最高的框加入结果集
3. 计算它和剩余所有框的 IoU（交并比）
4. 把 IoU > threshold 的框全部删除（它们是同一物体的重复检测）
5. 重复步骤 2~4，直到没有剩余框

---

## 十三、`Detect` 主函数

```cpp
bool Model::Detect(const cv::Mat &frame)
{
    try
    {
        this->detectResults.clear();

        preprocessing(frame);

        context->executeV2(buffers);
        cudaMemcpyAsync(this->prob.data(), this->buffers[1],
                        this->output_h * this->output_w * sizeof(float),
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

### 标准推理流水线

```text
1. clear()          清空上一轮结果
2. preprocessing()  图像预处理 + H2D 拷贝
3. executeV2()      TensorRT 推理（GPU 执行）
4. cudaMemcpyAsync  D2H 拷贝（GPU 输出 → CPU）
5. cudaStreamSynchronize  等待 GPU 完成
6. postprocessing() 解析结果
```

---

### `executeV2`

TensorRT 的同步推理接口。`buffers` 数组传入 GPU 显存地址，TensorRT 自动：

* 从 `buffers[0]` 读取输入
* 在 GPU 上执行前向传播
* 把输出写到 `buffers[1]`

`executeV2` 是 **v2 API**，相比旧的 `execute` 更简洁，不需要传入 batch size。

---

### `cudaStreamSynchronize`

阻塞 CPU，直到 `stream` 中的所有操作（包括 `executeV2` 和 `cudaMemcpyAsync`）全部完成。

这是**CPU-GPU 同步点**。在此之前，`prob` 中的数据还不完整，不能读。

---

### 异常处理

整个推理过程用 `try/catch` 包裹。如果 TensorRT 或 CUDA 出错（如显存不足、模型不兼容），打印错误信息并返回 `false`，但不崩程序。

---

## 十四、`predictClass` 分类接口

```cpp
int Model::predictClass(const cv::Mat &roi) {
    preprocessing(roi);
    context->executeV2(buffers);
    cudaMemcpyAsync(this->prob.data(), this->buffers[1],
                    this->output_h * this->output_w * sizeof(float),
                    cudaMemcpyDeviceToHost, this->stream);
    cudaStreamSynchronize(this->stream);
    float* data = (float*)prob.data();
    return std::max_element(data, data + (output_h * output_w)) - data;
}
```

---

### 和 `Detect` 的区别

| | `Detect` | `predictClass` |
|--|---------|---------------|
| 输入 | 整帧图像 | ROI 小图 |
| 预处理 | Letterbox + blob | 同上 |
| 后处理 | 解析 box + NMS | 直接取最大概率索引 |
| 返回值 | `bool`（是否检测到） | `int`（类别 ID） |

分类模型的输出就是一个概率向量（如 `[0.1, 0.7, 0.05, 0.15]`），不需要解析坐标，所以后处理极其简单：

```cpp
std::max_element(data, data + n) - data
```

找出最大值的索引，就是预测的类别。

---

# 十五、从 Model 层学到的设计要点

## 1. TensorRT 推理的生命周期管理

```text
Runtime → Engine → Context → Buffers → Stream
```

创建有顺序，释放有顺序，缺一不可。

## 2. 预处理的标准流程

```text
原图 → 等比例缩放 → 灰边填充 → 归一化 → HWC→CHW → H2D拷贝
```

这是所有基于 YOLO 的检测系统的通用做法。

## 3. 后处理的两种模式

| 模式 | 适用场景 | 特点 |
|------|---------|------|
| `isNMS=true` | 模型已做内置 NMS | 简单，直接解析 |
| `isNMS=false` | 原始 anchor 输出 | 需要手动 NMS，更灵活 |

## 4. 零拷贝技巧

```cpp
cv::Mat det_output(output_h, output_w, CV_32F, (float*)prob.data());
```

用 `cv::Mat` 包装已有内存，避免数据复制。

## 5. 异常隔离

```cpp
try { ... } catch (const std::exception& e) { ... }
```

单帧推理失败不崩程序，下一帧可以继续。
