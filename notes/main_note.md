# main.cpp

## 头文件

```cpp
#include <yaml-cpp/yaml.h>

#include "utils/include/draw.hpp"
```
`#include <yaml-cpp/yaml.h>`:引入`yaml-cpp`库的头文件，用于**解析和处理YAML配置文件**。

`#include "utils/include/draw.hpp"`:引入项目自定义的绘制工具头文件，用于**在图像上绘制目标检测结果。**

在项目中的作用：
加载` configs/detectConfig.yaml `配置文件

解析配置项（如模型路径、输入尺寸、置信度阈值等）

将配置值转换为 C++ 数据类型（如字符串、整数、浮点数等）

定义绘制函数（如 drawDetect）

实现检测结果的可视化（绘制边界框、类别标签等）


## 配置加载部分

此部分要和detectConfig.yaml一起看
```yaml
modelPath: /home/delphine/rm/tensorrt10_detect/models/best.engine # 模型路径

inputSize: 640 # 推理图像大小
scoreThreshold: 0.55 # 置信度阈值
nmsThreshold: 0.5 # 非极大值抑制阈值

# 类别列表
classNames:
  0: car
showFlag: true # 是否显示绘制并展示
```
```cpp
const YAML::Node modelConfig = YAML::LoadFile(modelConfig_path);​
```    

定义一个字符串（string）常量 `modelConfig_path`，并将其赋值为 YAML 配置文件的绝对路径。

`YAML::LoadFile(modelConfig_path)`:使用`yaml-cpp`库的`LoadFile`函数加载指定路径的 YAML 配置文件，并将其解析为一个 `YAML::Node` 对象。

`YAML::Node`是一个极其灵活的容器类，根据YAML文件中的内容，可以变身为以下几种形态：

- 映射（Map）：键值对的集合，类似于 C++ 中的 `std::map` 或 `std::unordered_map`。

- 序列（Sequence）：有序的元素列表，类似于 C++ 中的 `std::vector` 或 `std::list`。

- 标量（Scalar）：单个值，如字符串、整数、浮点数等。
   
当 `YAML::LoadFile` 执行后，`modelConfig` 这个变量就是一个根节点 (Root Node)。可以通过 [] 运算符像操作 C++ 的 `std::map` 一样去访问它的子节点。

`const YAML::Node modelConfig = `:声明一个不可修改的 `YAML::Node` 变量，存储加载后的配置数据。通过这个变量，程序可以访问和使用配置文件中的参数。

```cpp
const std::string modelConfig_path = "/home/delphine/rm/tensorrt10_detect/configs/detectConfig.yaml"; 
```
定义一个字符串（string）常量 `modelConfig_path`，并将其赋值为 YAML 配置文件的绝对路径。

这个路径指向项目中的 `configs/detectConfig.yaml` 文件，

该文件包含了模型的相关配置参数，如模型路径、输入尺寸、置信度阈值等。

通过这个路径，程序可以加载并解析配置文件，以便在后续的代码中使用这些配置参数。

```cpp
const std::string modelPath = modelConfig["modelPath"].as<std::string>(); 
```
`modelConfig["modelPath"]`:访问配置项，从 `modelConfig` 中获取键为 `"modelPath"`的配置项 

`.as<std::string>()`:将配置项的值转换为字符串类型

`const std::string modelPath = `:声明一个不可修改的字符串变量，存储**模型路径**。将转换后的配置值赋给变量

```cpp
const int inputSize = modelConfig["inputSize"].as<int>();  
```
`modelConfig["inputSize"]`:访问配置项，从 `modelConfig` 中获取键为 `"inputSize"`的配置项 

`.as<int>()`:将配置项的值转换为整数类型

`const int inputSize = `:声明一个不可修改的整数变量，存储**输入尺寸**。将转换后的配置值赋给变量

```cpp
const float scoreThreshold = modelConfig["scoreThreshold"].as<float>(); 
```
`modelConfig["scoreThreshold"]`:访问配置项，从 `modelConfig` 中获取键为 `"scoreThreshold"`的配置项 

`.as<float>()`:将配置项的值转换为浮点数类型

`const float scoreThreshold = `:声明一个不可修改的浮点变量，存储**置信度阈值**。将转换后的配置值赋给变量

```cpp
const float nmsThreshold = modelConfig["nmsThreshold"].as<float>();  
```
`modelConfig["nmsThreshold"]`:访问配置项，从 `modelConfig` 中获取键为 `"nmsThreshold"`的配置项 

`.as<float>()`:将配置项的值转换为浮点数类型

`const float nmsThreshold = `:声明一个不可修改的浮点变量，存储**NMS 阈值**。将转换后的配置值赋给变量

```cpp
const std::vector<std::string> classNames = []()
{
    std::vector<std::string> tmp;
    for (const auto &item : modelConfig["classNames"])
        tmp.emplace_back(item.second.as<std::string>());

    return tmp;
}();
```
- **Lambda**:`[]() { ... }`
  
  定义一个不可修改的字符串向量变量 `classNames`，用于存储**类别名称列表**。通过一个 lambda 函数来初始化该变量;

  在lambda内部，创建了`std::vector<std::string> tmp`，用于**临时存储类别名称。**

  通过循环遍历 `modelConfig["classNames"]` 中的每个项,将**YAML配置中每个类别名称**转换为字符串并添加到 `tmp` 向量中。
```
[捕获列表](参数列表) -> 返回类型 { 函数体 }
```


- **范围遍历**：c++11 引入的范围遍历（for-range loop），用于遍历容器（如向量、数组等）中的每个元素。

  无需管理索引，代码简洁，避免了索引越界，支持复杂容器，使用auto自动推导元素类型，无需显式指定。
```cpp
for (const auto &item : 容器/可迭代对象) {
    // 处理 item
}
```
**item**是一个键值对类型，其中`item.first`是类别索引（整数），`item.second`是类别名称（字符串）。

`item.second.as<std::string>()`:将`item.second`（类别名称）转换为字符串类型

`tmp.emplace_back()`将转换后的类别名称添加到 `tmp` 向量中。

最后，lambda函数返回 `tmp` 向量，并将其赋值给 `classNames` 变量，完成类别名称列表的初始化。

`()`运算符立即执行lambda表达式，将其返回值（类别名称列表）赋值给 `classNames` 变量。

在`drawDetect`函数中，使用`classNames`来绘制检测结果的类别标签。

模型检测到目标时，会返回类别id；通过`classNames`将类别id转换为对应的类别名称，以便在可视化结果中显示。

```cpp
const bool showFlag = modelConfig["showFlag"].as<bool>(); 
```
`modelConfig["showFlag"]`:访问配置项，从 `modelConfig` 中获取键为 `"showFlag"`的配置项 

`.as<bool>()`:将配置项的值转换为布尔类型

`const bool showFlag = `:声明一个不可修改的布尔变量，存储**是否显示可视化结果**的标志。将转换后的配置值赋给变量


``` cpp
char waitKey_Flag;
```
定义一个字符类型的变量 `waitKey_Flag`，用于存储用户按键输入的键值，作为退出程序的标志

## 主函数
```cpp
Model detectModel = Model(modelPath, inputSize, scoreThreshold, nmsThreshold);
bool detectFlag = false;
//读取并加载TensorRT引擎文件，初始化CUDA资源，获取模型输入输出维度，分配CUDA内存缓冲区
```
创建一个 `Model` 类型的对象 `detectModel`，使用以下配置参数初始化它。

- `modelPath`:模型路径，从配置文件中获取

- `inputSize`:输入尺寸，从配置文件中获取

- `scoreThreshold`:置信度阈值，从配置文件中获取

- `nmsThreshold`:NMS 阈值，从配置文件中获取

无返回值，直接初始化 `detectModel` 对象。

```cpp
while (true)
{
    cv::Mat frame = cv::imread("/home/delphine/rm/tensorrt10_detect/assets/02.png");
    double start_time = static_cast<double>(cv::getTickCount());
    
    detectFlag = detectModel.Detect(frame);

    // std::cout << (static_cast<double>(cv::getTickCount()) - start_time) / cv::getTickFrequency() << std::endl;
```

进入循环，持续读取图像帧进行检测。

使用 OpenCV 的 `cv::imread` 函数读取指定路径的图像文件，并将其存储在 `frame` 变量中。

记录当前时间，作为检测开始的时间点。

调用 `detectModel` 对象的 `Detect` 方法，对读取的图像帧进行目标检测，并将结果存储在 `detectFlag` 变量中。

```cpp
// 绘制, 展示检测结果
if (showFlag)
{
    drawDetect(frame, detectModel.detectResults, classNames);
    detectFlag = false;
    cv::imshow("img", frame);
    waitKey_Flag = cv::waitKey(1);
    if (waitKey_Flag == 113) // 
        break;
}
```
`showFlag`:是否显示可视化结果的标志，从配置文件中获取

如果 `showFlag` 为真，执行以下操作：

调用 `drawDetect` 函数，在图像帧上绘制检测结果。传入参数包括：

- `frame`:要绘制的图像帧
- `detectModel.detectResults`:检测结果数据，包含边界框、类别id、置信度等信息
- `classNames`:类别名称列表，用于绘制类别标签
- 将 `detectFlag` 变量重置为假，准备下一次检测
- `cv::imshow("img", frame)`:显示绘制了检测结果的图像帧，窗口标题为 `"img"`
- `waitKey_Flag = cv::waitKey(1)`:等待用户按键输入，1 毫秒超时。将按键值存储在 `waitKey_Flag` 变量中
- `if (waitKey_Flag == 113) // `:如果用户输入的键值为 `q`（ASCII 码为 113），则跳出循环，结束程序
```cpp
return 0;
```
程序正常结束，返回 0。

## 读取视频流版本的主函数
```cpp
int main(int argc, char const *argv[])
{
    Model detectModel = Model(modelPath, inputSize, scoreThreshold, nmsThreshold);
    bool detectFlag = false;
    
    // 打开视频文件
    cv::VideoCapture cap("/home/delphine/rm/tensorrt10_detect/assets/test_video.mp4");
    // 或使用摄像头: cv::VideoCapture cap(0);
    
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open video" << std::endl;
        return 1;
    }
    
    while (true)
    {
        cv::Mat frame;
        // 读取视频帧
        if (!cap.read(frame)) {
            // 视频读取完毕
            std::cout << "Video ended" << std::endl;
            break;
        }

        double start_time = static_cast<double>(cv::getTickCount());
        
        detectFlag = detectModel.Detect(frame);

        // std::cout << (static_cast<double>(cv::getTickCount()) - start_time) / cv::getTickFrequency() << std::endl;

        // 绘制, 展示检测结果
        if (showFlag)
        {
            drawDetect(frame, detectModel.detectResults, classNames);
            detectFlag = false;
            cv::imshow("img", frame);
            waitKey_Flag = cv::waitKey(1);
            if (waitKey_Flag == 113) // 'q' 键退出
                break;
        }
    }
    
    // 释放视频捕获资源
    cap.release();
    if (showFlag) {
        cv::destroyAllWindows();
    }

    return 0;
}

```

## tips
后续项目更加复杂时，考虑到可维护性，需要把配置加载逻辑，模型调用逻辑等从main.cpp中剥离

我们需要创建一个ConfigManager类，专门读取YAML

```cpp
#pragma once
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>

struct Config {
    std::string modelPath;
    int inputSize;
    float scoreThreshold;
    float nmsThreshold;
    std::vector<std::string> classNames;
    bool showFlag;

    Config(const std::string& path) {
        YAML::Node node = YAML::LoadFile(path);
        modelPath = node["modelPath"].as<std::string>();
        inputSize = node["inputSize"].as<int>();
        scoreThreshold = node["scoreThreshold"].as<float>();
        nmsThreshold = node["nmsThreshold"].as<float>();
        showFlag = node["showFlag"].as<bool>();
        for (auto it = node["classNames"].begin(); it != node["classNames"].end(); ++it) {
            classNames.push_back(it->second.as<std::string>());
        }
    }
};
```
经过改造后，我们发现main.cpp的逻辑变得更加清晰了
```cpp
#include "configs/config_manager.hpp" // 引入你的配置类
#include "utils/include/model.hpp"
#include "utils/include/draw.hpp"

int main(int argc, char const *argv[]) {
    // 1. 初始化配置 (一行代码搞定所有参数加载)
    Config cfg("/home/delphine/rm/tensorrt10_detect/configs/detectConfig.yaml");

    // 2. 初始化模型
    Model detectModel(cfg.modelPath, cfg.inputSize, cfg.scoreThreshold, cfg.nmsThreshold);

    // 3. 打开视频流
    cv::VideoCapture cap("/home/delphine/rm/car_project/test/02.mp4");
    if (!cap.isOpened()) return 1;

    cv::Mat frame;
    while (cap.read(frame)) {
        // 核心推理：这一行就是你的每一层逻辑入口
        detectModel.Detect(frame);

        // 绘制与展示
        if (cfg.showFlag) {
            drawDetect(frame, detectModel.detectResults, cfg.classNames);
            cv::imshow("img", frame);
            if (cv::waitKey(1) == 'q') break;
        }
    }
    
    cap.release();
    cv::destroyAllWindows();
    return 0;
}
```
**修改后的巨大优势：**

- main.cpp 的“整洁度”：

  以前全局变量占据了文件的 1/3，现在统统变成了 cfg.xxx。
  如果将来要加入 EfficientNet 分类，只需要在 Config 结构体里加一行 std::string effModelPath;，
  然后在 main 里实例化一个 Classifier 类即可。
- 解耦 (Decoupling)：
  
  以后不管模型逻辑怎么改（变三层架构、变四层架构），main.cpp 里的流程（读取帧 -> Detect -> 绘制）几乎不需要变动。
- 安全性：
 
  使用 Config 结构体存储配置，避免了全局变量被意外修改的风险。
 