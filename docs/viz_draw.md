# `draw.hpp` + `draw.cpp` 逐行讲解

这两份文件是项目的**检测可视化模块**，负责在图像上绘制检测框、类别标签和置信度分数。

---

# 第一部分：`draw.hpp`

## 一、头文件保护

```cpp
#ifndef __DRAW_HPP__
#define __DRAW_HPP__
```

标准 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include "model.hpp"
```

OpenCV 用于绘图，`model.hpp` 提供 `Result` 结构体和类别索引。

---

## 三、颜色表

```cpp
const std::vector<cv::Scalar> COLORS = {
    cv::Scalar(0, 0, 255),     // 红色
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(255, 0, 0),     // 蓝色
    cv::Scalar(255, 255, 0),   // 黄色
    cv::Scalar(0, 255, 255),   // 青色
    cv::Scalar(255, 0, 255),   // 洋红色
    cv::Scalar(255, 165, 0),   // 橙色
    cv::Scalar(128, 0, 128),   // 紫色
    cv::Scalar(0, 128, 128),   // 蓝绿色
};
```

---

### 为什么是 `const` 全局变量？

`const` 全局变量具有**内部链接性**，即使被多个 `.cpp` 文件包含，也不会产生重复定义错误。

每个类别用不同颜色绘制，方便肉眼区分不同目标。

---

### BGR 格式

OpenCV 默认 BGR 顺序：

| 颜色 | B | G | R |
|------|---|---|---|
| 红色 | 0 | 0 | 255 |
| 绿色 | 0 | 255 | 0 |
| 蓝色 | 255 | 0 | 0 |

---

## 四、`drawDetect` 声明

```cpp
void drawDetect(cv::Mat &frame, const std::vector<Result>& results,
                const std::vector<std::string> &classNames);
```

---

### 参数

| 参数 | 方向 | 含义 |
|------|------|------|
| `frame` | 输入/输出 | 要绘制的图像，绘制结果直接写回 |
| `results` | 输入 | 检测结果数组 |
| `classNames` | 输入 | 类别名称表，把数字 ID 映射成文字 |

---

# 第二部分：`draw.cpp`

## 一、`drawDetect` 实现

```cpp
void drawDetect(cv::Mat &frame, const std::vector<Result> &results,
                const std::vector<std::string> &classNames)
{
    for (const auto &result : results)
    {
```

遍历每个检测结果。

---

### 选择颜色

```cpp
        cv::Scalar color = COLORS.at(result.idx % COLORS.size());
```

用类别索引 `idx` 对颜色数组长度取模，选择颜色。

`% COLORS.size()` 确保即使 `idx` 超过颜色数量，也不会越界，而是循环使用颜色。

---

### 获取标签

```cpp
        cv::String label = classNames.at(result.idx % classNames.size());
```

从类别名称表中获取对应文字标签。同样用取模防止越界。

例如 `classNames` 可能是 `["car", "armor", "R1", "R2", "R3", "R4", "S"]`，`idx=2` 对应 `"R1"`。

---

### 绘制检测框

```cpp
        cv::rectangle(frame, result.box, color, 2);
```

用 `cv::rectangle` 画矩形框：

* `result.box`：`cv::Rect`，左上角 + 宽高
* `color`：框的颜色
* `2`：线宽 2 像素

---

### 绘制类别标签

```cpp
        cv::putText(frame, label,
                    cv::Point(result.box.x, result.box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
```

在检测框上方 10 像素处绘制类别名称。

* `cv::FONT_HERSHEY_SIMPLEX`：标准无衬线字体
* `1`：字体缩放系数
* `color`：文字颜色
* `2`：线宽

---

### 绘制置信度

```cpp
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << result.confidence;
        cv::putText(frame, ss.str(),
                    cv::Point(result.box.x + result.box.width / 2, result.box.y - 10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
```

---

#### `std::stringstream`

C++ 的字符串流，用于格式化输出。

---

#### `std::fixed` + `std::setprecision(2)`

固定小数点格式，保留 2 位小数。

```text
0.872345 → "0.87"
```

---

#### 置信度位置

放在检测框**上方中间**位置：

```cpp
cv::Point(result.box.x + result.box.width / 2, result.box.y - 10)
```

这样标签在左侧，置信度在右侧，不会重叠。

---

# 三、从 draw 层学到的设计要点

## 1. 取模防越界

```cpp
COLORS.at(result.idx % COLORS.size())
```

简洁优雅的防越界技巧。即使模型输出了超出预期的类别 ID，也不会崩溃。

## 2. 格式化输出

```cpp
std::stringstream ss;
ss << std::fixed << std::setprecision(2) << value;
```

C++ 标准库的格式化方式。C++20 后可以用 `std::format`，但 `stringstream` 兼容性最好。

## 3. 视觉层次

```text
框（最底层，定位目标）
  ↑
类别标签（左上角，说明是什么）
  ↑
置信度（上方中间，说明多确定）
```

三层信息分开放置，不重叠，人眼能快速读取。
