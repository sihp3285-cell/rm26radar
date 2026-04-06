# model.hpp
## 头文件
```cpp
#ifndef __MODEL_HPP__
#define __MODEL_HPP__
```
防止头文件重复包含，确保model.hpp只被编译一次

`#ifndef __MODEL_HPP__`：检查是否定义了 __MODEL_HPP__ 宏
`#define __MODEL_HPP__`：如果没有定义，则定义 __MODEL_HPP__ 宏
再次包含此头文件时，由于 __MODEL_HPP__ 已经定义，条件判断为假，不会执行后续代码，从而避免重复定义。

```cpp
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
```
引入必要的头文件，包括标准库、OpenCV、TensorRT等

- `<iostream>`:标准输入输出流库，用于打印日志和调试信息
- `<vector>`:动态数组容器库，用于存储检测结果等数据结构
- `<opencv2/opencv.hpp>`:OpenCV核心库，提供图像处理和计算机视觉功能
- `<opencv2/dnn.hpp>`:OpenCV深度学习模块头文件，提供加载和运行深度学习模型的功能
- `<NvInfer.h>`:TensorRT C++ API头文件，提供构建和运行TensorRT引擎的接口
- `<cuda_runtime_api.h>`:CUDA运行时API头文件，提供CUDA内存管理和设备操作的接口

## 结构体Result
```cpp
struct Result
{
    int idx;          // 类别编号
    float confidence; // 置信度
    cv::Rect box;     // 检测框
};
```
定义一个结构体Result，用于存储单个目标的检测结果，包括目标的类别，置信度，检测框位置信息。

(可以根据实际需求添加其他成员变量)
- `int idx`: 目标的类别编号，用于索引类别名称
- `float confidence`: 目标的置信度，范围[0, 1]，表示模型对目标的预测 certainty
- `cv::Rect box`: 目标的检测框，包含目标的位置信息 (x, y, width, height)
- `classIdx`（可选）用于分类结果，存储目标的分类类别编号
- `classConf`（可选）用于分类结果，存储目标的分类置信度

当模型执行目标检测后，每个检测到的目标都会被封装成一个`Result`对象

通过`confidence`与`scoreThreshold`比较，过滤低置信度的结果

基于`box`计算重叠度（IoU），与`nmsThreshold`比较，过滤重叠度高的结果

使用`box`绘制边界框，结合`idx`和`confidence`显示类别和置信度

## 类Model
```cpp
class Model
{
private:
    int inputSize;        // 模型输入图像尺寸
    float scoreThreshold; // 置信度阈值
    float nmsThreshold;   // 非极大值抑制阈值

    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    cudaStream_t stream = nullptr;
    void *buffers[2] = {nullptr, nullptr};
    std::vector<float> prob;

    int input_h, input_w;
    int output_h, output_w;
    float rx, ry;

    cv::Mat resizeFrame;

    void preprocessing(cv::Mat &frame);
    void postprocessing();

public:
    std::vector<Result> detectResults;

    Model(const std::string modelPath, const int &inputSize, const float &scoreThreshol, const float &nmsThreshold);
    ~Model();

    bool Detect(cv::Mat &frame);
};

#endif
```
`Model`类是项目的核心类，负责加载TensorRT模型、执行推理、后处理检测结果等功能

它封装了TensorRT和CUDA的底层操作，提供了简洁的接口用于目标检测。

```cpp
private:
    int inputSize;        // 模型输入图像尺寸
    float scoreThreshold; // 置信度阈值
    float nmsThreshold;   // 非极大值抑制阈值
```
  
模型的超参数：
- `inputSize`: 模型输入图像的尺寸，通常为正方形（如640x640）
- `scoreThreshold`: 置信度阈值，用于过滤低置信度的检测结果
- `nmsThreshold`: 非极大值抑制阈值，用于过滤重叠度高的检测框

```cpp
    nvinfer1::IRuntime *runtime = nullptr;
    nvinfer1::ICudaEngine *engine = nullptr;
    nvinfer1::IExecutionContext *context = nullptr;
    cudaStream_t stream = nullptr;
```

TensorRT推理核心：
- `IRuntime *runtime`: 加载引擎的工厂，它负责把硬盘上的`.engine`文件反序列化成内存中的引擎
- `ICudaEngine *engine`: 模型本身，包含模型的结构和权重。它是只读的，包含了所有算子的计算逻辑
- `IExecutionContext *context`: 执行上下文。这是唯一动态的部分。如果同一个引擎需要在多个线程中执行推理，每个线程都需要一个独立的执行上下文。它负责保存每一层的中间计算状态
- `cudaStream_t stream`: CUDA流，用于异步执行推理。每个执行上下文都有一个关联的流，用于在GPU上并行执行计算
```cpp
    void *buffers[2] = {nullptr, nullptr};
    std::vector<float> prob;
```
数据缓冲区与计算：
- `void *buffers[2]`: 输入和输出的GPU内存缓冲区指针数组。`buffers[0]`通常用于输入数据，`buffers[1]`用于输出结果。因为GPU无法直接读取CPU的内存，所以需要在GPU内存中分配缓冲区来存储输入和输出数据
- `std::vector<float> prob`: 用于存储从GPU拷回的输出数据，包含检测框的置信度和坐标信息。是连接GPU推理和CPU后处理的桥梁，确保在CPU上进行检测框解析和可视化等操作时，数据是最新的
```cpp
  int input_h, input_w;
    int output_h, output_w;
    float rx, ry;
```
辅助变量：
- `input_h`, `input_w`: 模型输入图像的高度和宽度，通常与`inputSize`相同
- `output_h`, `output_w`: 输出特征图高度宽度，如类别数，检测框数量
- `rx`, `ry`: 用于将检测框坐标从模型输出空间映射回原始输入图像空间，用于调整检测框的实际坐标
```cpp
cv::Mat resizeFrame;

    void preprocessing(cv::Mat &frame);
    void postprocessing();​
```
处理函数：
- `cv::Mat resizeFrame`: 用于存储预处理后的图像，尺寸为`inputSize`
- `preprocessing(cv::Mat &frame)`: 输入图像预处理函数，负责将输入图像调整为模型要求的尺寸和格式，进行归一化等操作
- `postprocessing()`: 后处理函数，负责将类内部存储的模型输出数据解析为检测结果，包括置信度过滤，非极大值抑制，坐标映射等操作
```cpp
public:
    std::vector<Result> detectResults;

    Model(const std::string modelPath, const int &inputSize, const float &scoreThreshol, const float &nmsThreshold);
    ~Model();

    bool Detect(cv::Mat &frame);
```
检测结果存储：
- `std::vector<Result> detectResults`: 存储当前帧的检测结果，每个`Result`对象包含一个检测目标的信息，如类别索引、置信度、检测框坐标等
用于在`postprocessing`（详见model.cpp）中解析模型输出，填充`detectResults`，向外部（如`main.cpp`）提供检测结果，用于绘制和展示，每次调用`Detect`后，`detectResults`会被清空，准备存储新的检测结果

构造函数和析构函数：

- `Model(const std::string modelPath, const int &inputSize, const float &scoreThreshol, const float &nmsThreshold);`:初始化`Model`对象，加载TensorRT模型、创建执行上下文、分配GPU内存等,准备推理环境
- `~Model();`:释放`Model`对象占用的资源，销毁执行上下文、引擎、CUDA流等，确保没有内存泄漏

检测方法：
- `bool Detect(cv::Mat &frame);`: 检测方法，输入原始图像`frame`，返回是否成功检测到目标