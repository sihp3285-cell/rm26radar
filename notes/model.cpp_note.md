# model.cpp
## 头文件
```cpp
#include <fstream>
#include <algorithm>
#include <numeric>
#include "include/model.hpp"
```
引入必要的头文件，包括标准库和model.hpp
- `<fstream>`:文件流库，用于读取TensorRT模型文件和配置文件
- `<algorithm>`:算法库，提供排序、查找等常用算法,用于NMS非极大值抑制，过滤检测结果等
- `<numeric>`:数值库，提供数值计算函数，如累加、累乘等，用于计算检测框的面积、IOU等
- `"include/model.hpp"`：自定义头文件，包含模型类Model，结构体Result的定义，以及必要的库依赖
## Logger类
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
这里拆分成四个部分进行理解
```cpp
class Logger : public nvinfer1::ILogger
```
`nvinfer1::ILogger`是NVIDIA TensorRT定义的一个抽象基类（接口）
TensorRT库本身不知道程序会用什么方式显示信息（控制台？文件？）所以它定义了一个接口`ILogger`，要求我们必须实现一个`log`函数，当TensorRT库发生警告或报错，需要输出日志时，会自动调用这个函数（回调机制）
```cpp
{
void log(Severity severity, const char *msg) noexcept override
```
这是`ILogger`接口中定义的纯虚函数，我们必须实现它。这个函数接受两个参数：
- `Severity severity`:日志级别，用于区分不同类型的日志（如警告、错误、信息等）
- `const char *msg`:日志消息，即要输出的文本内容
 
`override`是一个C++的关键字，为了告诉编译器“这是一个重写（覆盖）父类的函数”，如果父类没有这个函数，编译器会报错，防止我们写错接口

`noexcept`是一个C++的关键字，用于告诉编译器“这个函数不会抛出异常”，这是库接口设计的最佳实践，防止异常跨越库边界导致程序直接崩溃
```cpp
{
if (severity <= Severity::kINFO)
            std::cout << msg << std::endl;
}
```
这是一个过滤器。由于TensorRT产生的信息非常多。此处的if判断是为了屏蔽掉过于详细的调试信息，只输出警告、错误和一般信息。保证控制台的简洁
```cpp
} gLogger;
```
实例化一个全局对象`gLogger`，用于TensorRT库的日志输出。TensorRT需要传入一个Logger对象的引用，以便在运行时输出日志信息。整个程序只需要一个"汇报官"，故而定义为全局对象可以方便在任何地方进行调用
## Model::Model构造函数
```cpp
Model::Model(const std::string modelPath, const int &inputSize, const float &scoreThreshold, const float &nmsThreshold)
{
    this->inputSize = inputSize;
    this->scoreThreshold = scoreThreshold;
    this->nmsThreshold = nmsThreshold;

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

    this->runtime = nvinfer1::createInferRuntime(gLogger);
    assert(this->runtime != nullptr);

    this->engine = this->runtime->deserializeCudaEngine(engineData.data(), fsize);
    assert(this->engine != nullptr);

    this->context = this->engine->createExecutionContext();
    assert(this->context != nullptr);

    // 获取输入输出维度
    nvinfer1::Dims inputDims = this->engine->getTensorShape("images");
    nvinfer1::Dims outputDims = this->engine->getTensorShape("output0");

    this->input_h = inputDims.d[2];
    this->input_w = inputDims.d[3];

    this->output_h = outputDims.d[1];
    this->output_w = outputDims.d[2];

    cudaMalloc(&(this->buffers[0]), this->input_h * this->input_w * 3 * sizeof(float));
    cudaMalloc(&(this->buffers[1]), this->output_h * this->output_w * sizeof(float));

    this->prob.resize(this->output_h * this->output_w);

    // Create stream
    cudaStreamCreate(&(this->stream));
}
```
此处是TensorRT部署中最核心的模型初始化阶段
```cpp
    this->inputSize = inputSize;
    this->scoreThreshold = scoreThreshold;
    this->nmsThreshold = nmsThreshold;
```
将外部传入的配置参数赋值给类的成员变量，完成模型的初始化。为后续的推理提供基础配置。
```cpp
    std::ifstream engineFile(modelPath, std::ios::binary);
    std::vector<char> engineData;
    int fsize = 0;
```
创建一个输入文件流，以二进制模式打开TensorRT引擎文件。
- `std::ifstream`是C++标准库中的文件流类，用于读取文件。
- `modelPath`是传入的TensorRT引擎文件路径，
- `std::ios::binary`表示以二进制模式打开文件，确保不会因为换行符等问题导致读取错误。
 
创建一个动态字符数组`engineData`，用于存储引擎文件的内容。作为缓冲区，存储整个引擎文件的内容。便于后续传递给TensorRT库进行反序列化。

声明并初始化一个整数变量`fsize`，用于存储引擎文件的大小（字节数）。确定后续反序列化时需要读取多少字节,为engineData分配足够的内存空间。
```cpp
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
如果文件打开成功（`engineFile.good()`返回true），则执行以下操作：
- 将文件指针移动到文件末尾（`engineFile.seekg(0, engineFile.end)`），获取文件大小（`fsize = engineFile.tellg()`）。
- 将文件指针移动回文件开头（`engineFile.seekg(0, engineFile.beg)`）。
- 调整`engineData`的大小为`fsize`，确保有足够的空间存储引擎文件内容。
- 读取文件内容到`engineData`中（`engineFile.read(engineData.data(), fsize)`）。
- 关闭文件流（`engineFile.close()`）。

`seekg`：全称为`seek to get`，用于将文件指针移动到指定位置。（`seekg(offset, from)`）
- `offset`：偏移量，即要移动的字节数。可以是正数（向文件末尾移动）或负数（向文件开头移动）。
- `from`：指定从哪个位置开始移动指针。可以是`std::ios::beg`（文件开头）、`std::ios::cur`（当前位置）或`std::ios::end`（文件末尾）。
`tellg`：全称为`tell position get`，用于获取当前文件指针的位置（即从文件开头开始的字节数）。（`tellg()`）

`resize`：用于调整容器的大小。（`resize(size)`）
- `size`：新的容器大小。如果新大小大于当前大小，则会在容器末尾添加默认值（如0）。如果新大小小于当前大小，则会删除超出部分的元素。

`read`：用于从文件流中读取数据到指定的内存地址。（`read(data, size)`）
- `data`：指向要存储读取数据的内存地址的指针。
- `size`：要读取的字节数。
 
`close`：用于关闭文件流。（`close()`）
```cpp
    this->runtime = nvinfer1::createInferRuntime(gLogger);
    assert(this->runtime != nullptr);
```
创建TensorRT推理运行时对象`runtime`，用于加载和管理TensorRT引擎。传入全局Logger对象`gLogger`以便输出日志信息（强制）

运行时：TensorRT的基础组件，负责加载和管理TensorRT引擎。

`assert`的意义：assert是一个防御性编程工具，如果createInferRuntime因为系统资源耗尽（没有驱动，没权限等），返回了nullptr，会触发断言失败，提示"createInferRuntime failed"。
```cpp
    this->engine = this->runtime->deserializeCudaEngine(engineData.data(), fsize);
    assert(this->engine != nullptr);
```
将内存中的engineData二进制数据，转化为TensorRT能理解的“模型对象”
- engineData.data()：指向engineData中存储引擎文件内容的内存地址的指针。
- fsize：引擎文件的大小（字节数）。

序列化：将模型对象转换为二进制数据，以便后续存储或传输。

反序列化：将二进制数据转换回模型对象，以便在内存中使用。

此处即为TensorRT模型的反序列化过程，将引擎文件内容转换为TensorRT能理解的模型对象，存储在`this->engine`中。
```cpp
    this->context = this->engine->createExecutionContext();
    assert(this->context != nullptr);
```

创建TensorRT执行上下文对象`context`，用于执行推理。每个执行上下文都是独立的，可以在多线程环境中并行执行同一个引擎的推理。
- `engine`是只读的，是模型的本体
- `context`是可读写的，用于存储推理时的中间结果和状态。

同一个engine可以对应多个context，每个context都是独立的，用于在不同线程中执行推理。如果需要在多个线程中并行执行推理，只需为每个线程创建一个独立的context。它们都共享同一个engine

```cpp
    nvinfer1::Dims inputDims = this->engine->getTensorShape("images");
    nvinfer1::Dims outputDims = this->engine->getTensorShape("output0");
```
获取TensorRT引擎中输入和输出张量的维度信息
- `nvinfer1::Dims`：TensorRT库中用于表示张量维度的结构体。
- `inputDims`：存储输入张量的维度信息（如通道数、高度、宽度等）。
- `outputDims`：存储输出张量的维度信息（如类别数、高度、宽度等）。
- `getTensorShape`：用于获取指定张量的维度信息。（`getTensorShape(tensorName)`）
- `tensorName`：张量的名称（如"images"(在模型定义或转换时指定)、"output0"（YOLO通常使用）等）
 
后续中用于提取具体维度值，如通道数、高度、宽度等。计算内存需求，调整输出存储等
```cpp
    this->input_h = inputDims.d[2];
    this->input_w = inputDims.d[3];

    this->output_h = outputDims.d[1];
    this->output_w = outputDims.d[2];
```
根据输入输出张量的维度信息，提取具体的高度和宽度值

TensorRT使用NCHW维度顺序（批次，通道，高度，宽度）：
- `inputDims.d[0]`：输入张量的批次大小（通常为1）
- `inputDims.d[1]`：输入张量的通道数（如3）
- `inputDims.d[2]`：输入张量的高度
- `inputDims.d[3]`：输入张量的宽度
以YOLO模型为例，输出张量维度通常为【1，84，8400】（1批次，84通道，8400检测框）
- `outputDims.d[1]`：输出张量的高度（对应通道数，包括类别和坐标信息）
- `outputDims.d[2]`：输出张量的宽度（对应检测框数量）

后续用于内存分配，输出存储调整，图像预处理，结果后处理等

```cpp
    cudaMalloc(&(this->buffers[0]), this->input_h * this->input_w * 3 * sizeof(float));
    cudaMalloc(&(this->buffers[1]), this->output_h * this->output_w * sizeof(float));
```
在GPU上为输入和输出数据分配内存缓冲区
- `cudaMalloc`：CUDA函数，用于在GPU上分配内存。
- `&(this->buffers[0])`：
- - `this->buffers[0]`：指向输入数据缓冲区的指针。
- - `&` ：取地址运算符，用于获取`this->buffers[0]`的内存地址，作为`cudaMalloc`的参数。
- 

- `this->input_h * this->input_w * 3 * sizeof(float)`:计算出输入张量在内存中占用的总字节数。
- -  `this->input_h * this->input_w * 3`：输入张量的总元素数（高度 * 宽度 * 通道数）
- - `sizeof(float)`：每个元素占用4字节（32位浮点数）
- 后者同理，为计算输出张量在内存中占用的总字节数。在GPU上分配对应大小的内存缓冲区。
```cpp
    this->prob.resize(this->output_h * this->output_w);
```
调整`prob`向量的大小以存储从GPU拷回的输出数据。`output_h * output_w`对应输出张量中的检测框的数量，每个检测框包含类别置信度和坐标信息。
```cpp
    cudaStreamCreate(&(this->stream));
```
创建一个CUDA流，用于异步执行推理。每个执行上下文都有一个关联的流，用于在GPU上并行执行计算。
## Model::~Model() ：Model类的析构函数
```cpp
Model::~Model()
{
    // 释放资源
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
在Model类的析构函数中，我们需要释放所有分配的资源，确保没有内存泄漏
```cpp
for (auto &buffer : this->buffers)
{
    if (buffer)
        cudaFree(buffer);
}
```
`for (auto &buffer : this->buffers)`：C++11的范围循环，遍历`buffer[0]`(输入)和`buffer[1]`(输出)

`if (buffer)`：检查当前缓冲区指针是否非空，确保只释放已经分配的内存。
`cudaFree(buffer)`：CUDA显存释放API，用于释放之前在GPU上分配的内存。
```cpp
    if (this->context)
        delete this->context;
    if (this->engine)
        delete this->engine;
    if (this->runtime)
        delete this->runtime;
```
按照创建的相反顺序，依次销毁执行上下文、TensorRT引擎、TensorRT运行时，释放所有相关内存资源。

并且确保只释放非空指针，避免悬空指针错误。
```cpp
    if (this->stream)
        cudaStreamDestroy(this->stream);
```
销毁CUDA流，释放相关资源。
## Model::preprocessing(cv::Mat &frame) ：图像预处理函数
```cpp
// 旧版 强制缩放
void Model::preprocessing(cv::Mat &frame)
{
    float frame_width = frame.cols;
    float frame_height = frame.rows;

    float r = float(this->inputSize / std::max(frame_width, frame_height));

    this->rx = frame_width / this->inputSize;
    this->ry = frame_height / this->inputSize;

    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);
    cudaMemcpyAsync(buffers[0], blob.ptr<float>(), 3 * this->input_h * this->input_w * sizeof(float), cudaMemcpyHostToDevice, this->stream);
}
```
该函数负责将输入图像进行预处理，使其符合TensorRT模型的输入要求，并将处理后的数据传输到GPU内存中，为后续模型推理做准备
```cpp
    float frame_width = frame.cols;
    float frame_height = frame.rows;

    float r = float(this->inputSize / std::max(frame_width, frame_height));

    this->rx = frame_width / this->inputSize;
    this->ry = frame_height / this->inputSize;
```
目的：为了后续将模型预测出的目标框坐标从“模型输入尺寸”映射为“原始图像尺寸”
- `frame_width`和`frame_height`：获取输入图像的宽度和高度。
- `this->inputSize`：模型输入尺寸（通常为416或608）
- `std::max(frame_width, frame_height)`：计算输入图像的较长边。
- `this->inputSize / std::max(frame_width, frame_height)`：计算缩放比例，将较长边缩放到`inputSize`。
- `float(r)`：将缩放比例转换为浮点数，确保后续计算的精度。
- `r`：缩放比例，将较长边缩放到`inputSize`。
- `this->rx = frame_width / this->inputSize`：计算图像在x轴上的缩放因子，将模型输出的框坐标映射回原始图像。
- `this->ry = frame_height / this->inputSize`：计算图像在y轴上的缩放因子，将模型输出的框坐标映射回原始图像。
- 这里的`this->rx`和`this->ry`计算逻辑取决于模型后续如何处理图像（直接拉伸？通过letterbox添加黑边？）如果后续没有使用r缩放图像，这个rx/ry仅代表直接拉伸的情况
```cpp
cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);
```
使用OpenCV的`blobFromImage`函数将输入图像转换为TensorRT模型所需的blob格式。
- `frame`：输入图像（OpenCV Mat格式）
- `1 / 255.0`：归一化因子，将像素值从[0, 255]范围映射到[0, 1]范围
- `cv::Size(this->input_w, this->input_h)`：目标blob尺寸，与模型输入尺寸匹配
- `cv::Scalar(0, 0, 0)`：均值减法值，这里设为0表示不进行均值减法
- `true`：是否交换通道顺序，这里设为true表示将BGR通道顺序转换为RGB顺序
- `false`：是否不进行归一化，这里设为false表示进行归一化
- 转换后的`blob`是一个NCHW格式的4维矩阵，即【1，通道数，高，宽】
```cpp
cudaMemcpyAsync(buffers[0], blob.ptr<float>(), 3 * this->input_h * this->input_w * sizeof(float), cudaMemcpyHostToDevice, this->stream);
```
将预处理后的blob数据异步复制到GPU内存中的输入缓冲区。
- `buffers[0]`：指向GPU输入缓冲区的指针。
- `blob.ptr<float>()`：获取`blob`在CPU内存中的起始地址
- `3 * this->input_h * this->input_w * sizeof(float)`：计算需要拷贝的数据字节总数
- `cudaMemcpyHostToDevice`：指定从主机内存（CPU）复制到设备内存（GPU）
- `this->stream`：指定异步操作的CUDA流，用于并行执行其他CUDA操作
```cpp
//新版 letterbox 保持长宽比
void Model::preprocessing(cv::Mat &frame)
{
    int img_w = frame.cols;
    int img_h = frame.rows;

    // 1. 计算缩放比例 (保持长宽比)
    float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
    int new_w = int(img_w * scale);
    int new_h = int(img_h * scale);

    // 2. 缩放图像
    cv::Mat resized_img;
    cv::resize(frame, resized_img, cv::Size(new_w, new_h));

    // 3. 创建画布并填充颜色 (通常用 114, 114, 114 或 0, 0, 0)
    cv::Mat canvas = cv::Mat::zeros(this->input_h, this->input_w, CV_8UC3);
    canvas.setTo(cv::Scalar(114, 114, 114)); // 填充灰色，很多YOLO模型习惯用这个颜色
    
    // 将缩放后的图贴入画布中心
    resized_img.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));

    // 4. 计算还原坐标的比例 (用于推理后把框放回原图)
    this->rx = (float)img_w / new_w;
    this->ry = (float)img_h / new_h;

    // 5. 转换为 Blob (此时 canvas 大小已经固定为 input_w x input_h)
    // 归一化到 [0, 1]，BGR 转 RGB
    cv::Mat blob = cv::dnn::blobFromImage(canvas, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);
    
    // 6. 拷贝到 GPU
    cudaMemcpyAsync(buffers[0], blob.ptr<float>(), 3 * this->input_h * this->input_w * sizeof(float), cudaMemcpyHostToDevice, this->stream);
}
```
这是一个改进版的预处理函数，采用了letterbox缩放方法，保持图像的长宽比不变，并在缩放后的图像周围添加填充，使其适应模型输入尺寸。

这样可以避免直接拉伸图像导致的变形问题，提高检测精度。
```cpp
int img_w = frame.cols;
int img_h = frame.rows;

// 1. 计算缩放比例 (保持长宽比)
float scale = std::min((float)this->input_w / img_w, (float)this->input_h / img_h);
int new_w = int(img_w * scale);
int new_h = int(img_h * scale);
```
- 获取输入图像的宽度和高度，分别赋值给`img_w`和`img_h`。
- 计算缩放比例`scale`，通过比较输入图像的宽高与模型输入尺寸的比例，选择较小的那个作为缩放比例，以保持图像的长宽比不变。
- 根据计算得到的缩放比例，计算缩放后的图像尺寸`new_w`和`new_h`。
```cpp
// 2. 缩放图像
cv::Mat resized_img;
cv::resize(frame, resized_img, cv::Size(new_w, new_h));

// 3. 创建画布并填充颜色 (通常用 114, 114, 114 或 0, 0, 0)
cv::Mat canvas = cv::Mat::zeros(this->input_h, this->input_w, CV_8UC3);
canvas.setTo(cv::Scalar(114, 114, 114)); // 填充灰色，很多YOLO模型习惯用这个颜色

// 将缩放后的图贴入画布中心
resized_img.copyTo(canvas(cv::Rect(0, 0, new_w, new_h)));
```
- 使用OpenCV的`resize`函数将输入图像缩放到新的尺寸`new_w`和`new_h`，得到缩放后的图像`resized_img`。
- 创建一个新的画布`canvas`，大小为模型输入尺寸，类型为8位无符号整型（CV_8UC3），用于存储最终的预处理图像。
- 使用`setTo`函数将画布填充为灰色（114, 114, 114），这是很多YOLO模型常用的填充颜色。
- 使用`copyTo`函数将缩放后的图像`resized_img`贴入画布中心，确保图像居中对齐。
```cpp
// 4. 计算还原坐标的比例 (用于推理后把框放回原图)
this->rx = (float)img_w / new_w;
this->ry = (float)img_h / new_h;
```
- 计算坐标还原比例`rx`和`ry`，用于将模型输出的检测框坐标从缩放后的图像空间映射回原始图像空间。
```cpp
// 5. 转换为 Blob (此时 canvas 大小已经固定为 input_w x input_h)
// 归一化到 [0, 1]，BGR 转 RGB
cv::Mat blob = cv::dnn::blobFromImage(canvas, 1 / 255.0, cv::Size(this->input_w, this->input_h), cv::Scalar(0, 0, 0), true, false);

// 6. 拷贝到 GPU
cudaMemcpyAsync(buffers[0], blob.ptr<float>(), 3 * this->input_h * this->input_w * sizeof(float), cudaMemcpyHostToDevice, this->stream);

```
- 使用`blobFromImage`函数将预处理后的画布图像转换为模型输入所需的blob格式，进行归一化和通道转换。
- 归一化到 [0, 1]，将图像通道从 BGR 转换为 RGB 格式。
- 将转换后的blob数据异步复制到GPU内存中的输入缓冲区`buffers[0]`。
- 使用CUDA流`this->stream`进行异步操作，允许同时执行其他CUDA任务，提高效率。
## Model:: postprocessing后处理
```cpp
void Model::postprocessing()
{
    std::vector<cv::Rect> boxes;
    std::vector<int> classIds;
    std::vector<float> confidences;
    cv::Mat det_output(this->output_h, this->output_w, CV_32F, (float *)prob.data());

    cv::Rect box;
    double score;
    cv::Point class_id_point;
    cv::Mat classes_scores;

    for (int idx = 0; idx < det_output.cols; ++idx)
    {
        classes_scores = det_output.col(idx).rowRange(4, this->output_h);
        cv::minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id_point);

        if (score > this->scoreThreshold)
        {
            const float cx = det_output.at<float>(0, idx);
            const float cy = det_output.at<float>(1, idx);
            const float ow = det_output.at<float>(2, idx);
            const float oh = det_output.at<float>(3, idx);

            box.x = static_cast<int>((cx - 0.5 * ow) * this->rx);
            box.y = static_cast<int>((cy - 0.5 * oh) * this->ry);
            box.width = static_cast<int>(ow * this->rx);
            box.height = static_cast<int>(oh * this->ry);

            boxes.push_back(box);
            classIds.push_back(class_id_point.y);
            confidences.push_back(score);
        }
    }

    std::vector<int> indexes;
    cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, indexes);

    this->detectResults.reserve(indexes.size()); // 预分配空间
    for (int idx : indexes)
    {
        int classId = classIds.at(idx);
        this->detectResults.emplace_back(Result{classId, confidences.at(idx), boxes.at(idx)});
    }
}
```
该函数负责将模型推理得到的输出数据进行后处理，提取检测框、类别和置信度，

应用非极大值抑制（NMS）过滤掉重复的检测结果，最终得到最终的检测结果列表。
```cpp
std::vector<cv::Rect> boxes;
std::vector<int> classIds;
std::vector<float> confidences;
cv::Mat det_output(this->output_h, this->output_w, CV_32F, (float *)prob.data());
```
创建存储容器：
- `boxes`：用于存储检测框的坐标。
- `classIds`：用于存储检测框所属的类别ID。
- `confidences`：用于存储检测框的置信度分数。
- `det_output`：将模型输出数据转换为32位浮点数的OpenCV矩阵，用于后续的处理。
- - `this->output_h`：模型输出的高度（每个检测框包含的信息维度），用于创建`det_output`矩阵。
- - - 前 4 行：代表边界框的坐标信息。
- - - [0]：中心点 X 坐标 (cx)
- - - [1]：中心点 Y 坐标 (cy)
- - - [2]：框的宽度 (ow)
- - - [3]：框的高度 (oh)
- - - 后续行：代表类别得分。
- - - 如果模型有 80 个类别（例如 COCO 数据集），则 output_h 应该是 4 + 80 = 84。
- - `this->output_w`：模型输出的宽度（检测框数量），用于创建`det_output`矩阵。
- - `prob.data()`：模型输出数据的指针，用于初始化`det_output`矩阵。
```cpp
for (int idx = 0; idx < det_output.cols; ++idx)
{
    classes_scores = det_output.col(idx).rowRange(4, this->output_h);
    cv::minMaxLoc(classes_scores, nullptr, &score, nullptr, &class_id_point);

    if (score > this->scoreThreshold)
    {
        const float cx = det_output.at<float>(0, idx);
        const float cy = det_output.at<float>(1, idx);
        const float ow = det_output.at<float>(2, idx);
        const float oh = det_output.at<float>(3, idx);

        box.x = static_cast<int>((cx - 0.5 * ow) * this->rx);
        box.y = static_cast<int>((cy - 0.5 * oh) * this->ry);
        box.width = static_cast<int>(ow * this->rx);
        box.height = static_cast<int>(oh * this->ry);

        boxes.push_back(box);
        classIds.push_back(class_id_point.y);
        confidences.push_back(score);
    }
}
```
遍历模型输出的每个检测框（列），提取类别置信度和坐标信息。

由于我们是按列遍历的，所以`idx`代表当前检测框的索引。

`det_output.col(idx).rowRange(4, this->output_h)`：从模型输出中提取当前检测框的类别置信度行，范围从第4行到输出高度。（`rowRange(4, this->output_h)`就是把前四行坐标信息砍掉，只留下类别分数部分）

使用`cv::minMaxLoc`函数（用于在给定的矩阵中找到最大/最小值的位置）找到当前检测框的最高类别置信度和对应的类别ID。

如果最高类别置信度超过设定的`scoreThreshold`，则认为这是一个有效的检测结果，继续提取坐标信息。过滤掉低于阈值的检测框。

根据模型输出的坐标信息（中心点坐标和宽高），计算检测框的左上角坐标和宽高，将其映射回原始图像空间。

将有效的检测结果存储到`boxes`、`classIds`和`confidences`容器中。
```cpp
std::vector<int> indexes;
cv::dnn::NMSBoxes(boxes, confidences, this->scoreThreshold, this->nmsThreshold, indexes);
```
非极大值抑制（NMS）去除重叠的检测框：
- `boxes`：检测框的坐标容器。
- `confidences`：检测框的置信度容器。
- `this->scoreThreshold`：置信度阈值，低于该值的检测框将被过滤掉。
- `this->nmsThreshold`：NMS 阈值，用于抑制重叠检测框。
- `indexes`：输出容器，存储通过 NMS 过滤后的检测框索引。
```cpp
this->detectResults.reserve(indexes.size()); // 预分配空间
for (int idx : indexes)
{
    int classId = classIds.at(idx);
    this->detectResults.emplace_back(Result{classId, confidences.at(idx), boxes.at(idx)});
}
```
预分配空间，为`detectResults`预分配内存，提高性能

遍历通过NMS过滤后的检测框索引，从`classIds`、`confidences`和`boxes`容器中提取对应的类别ID、置信度和坐标信息，创建`Result`对象，并将其添加到最终的检测结果列表`detectResults`中。

`Result`：包含以下成员变量：
- `classId`：检测框所属的类别ID。
- `confidence`：检测框的置信度分数。
- `box`：检测框的坐标信息，包含左上角坐标和宽高。
## Model::Detect检测函数
```cpp
bool Model::Detect(cv::Mat &frame)
{
    try
    {
        this->detectResults.clear();

        preprocessing(frame);

        context->executeV2(buffers);
        cudaMemcpyAsync(this->prob.data(), this->buffers[1], this->output_h * this->output_w * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
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
该函数是Model类的核心接口，负责执行完整的检测流程，包括预处理、推理和后处理，并返回是否成功检测到目标。
```cpp
try
{
    this->detectResults.clear();

```
使用`try-catch`块捕获可能在检测过程中发生的异常，确保程序的健壮性。
- `this->detectResults.clear()`：在每次检测前清空之前的检测结果列表，确保每次检测都是独立的。
```cpp
preprocessing(frame);
```
调用预处理函数，对输入图像进行预处理，使其符合模型输入要求。预处理后的数据存储在GPU输入缓冲区`buffers[0]`中。
```cpp
context->executeV2(buffers);
```
调用TensorRT执行上下文的`executeV2`函数，执行模型推理。推理结果存储在GPU输出缓冲区`buffers[1]`中。
```cpp
cudaMemcpyAsync(this->prob.data(), this->buffers[1], this->output_h * this->output_w * sizeof(float), cudaMemcpyDeviceToHost, this->stream);
cudaStreamSynchronize(this->stream);
```
使用`cudaMemcpyAsync`函数将GPU输出缓冲区`buffers[1]`中的数据异步复制到主机内存`this->prob.data()`中。
- `this->prob.data()`：指向主机内存中存储推理结果的指针。
- `this->buffers[1]`：指向GPU输出缓冲区的指针，存储模型推理结果。
- `this->output_h * this->output_w * sizeof(float)`：推理结果的字节数，等于输出高度、输出宽度和每个元素的字节数（float类型为4字节）的乘积。
- `cudaMemcpyDeviceToHost`：指定从设备（GPU）内存复制到主机（CPU）内存。
- `this->stream`：使用的CUDA流，用于异步执行内存复制操作。
 
同步等待：使用`cudaStreamSynchronize`函数同步等待CUDA流`this->stream`中的所有操作完成。
```cpp
postprocessing();
```
调用后处理函数，对模型推理结果进行处理，提取检测框、类别和置信度，并应用非极大值抑制（NMS）过滤掉重复的检测结果，最终得到检测结果列表`detectResults`。
```cpp
return !detectResults.empty();
```
返回检测状态，即是否成功检测到目标。如果`detectResults`列表不为空，则返回`true`，表示成功检测到目标；否则返回`false`，表示未检测到目标。
```cpp
catch (const std::exception &e)
{
    std::cerr << e.what() << '\n';
    return false;
}
```
捕获检测过程中可能发生的异常，并输出异常信息到标准错误流，确保程序不会因为未处理的异常而崩溃，并返回`false`表示检测失败。