# `mouseback.hpp` + `mouseback.cpp` 逐行讲解

这两份文件实现了一个简单的**交互式标定工具**：在 OpenCV 窗口中显示图像，让用户用鼠标点击标定点，收集像素坐标供 PnP 解算使用。

---

# 第一部分：`mouseback.hpp`

## 一、头文件保护

```cpp
#ifndef MOUSEBACK_HPP
#define MOUSEBACK_HPP
```

标准 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include <vector>
```

OpenCV 用于窗口和鼠标回调，`vector` 存储点集。

---

## 三、`MouseBack` 类

```cpp
class MouseBack
{
```

---

### 私有成员

```cpp
private:
    std::vector<cv::Point2f> points;   // 收集到的标定点
    std::string windowName;            // 窗口名称
    int maxpoints;                     // 需要收集的最大点数
    static void onMouse(int event, int x, int y, int flags, void* userdata);
```

---

#### `onMouse`

**静态成员函数**，作为 OpenCV 鼠标回调函数。

为什么必须是 `static`？因为 OpenCV 的 `cv::setMouseCallback` 要求传入 C 风格的函数指针：

```cpp
typedef void (*MouseCallback)(int event, int x, int y, int flags, void* userdata);
```

非静态成员函数有隐式的 `this` 指针，签名不匹配，不能传给 C API。静态成员函数没有 `this`，签名匹配。

---

### 公共接口

```cpp
public:
    MouseBack(const std::string& windowName, int requirePoints);
    std::vector<cv::Point2f> getPoints(const cv::Mat& frame);
```

---

#### `getPoints`

核心接口。显示图像窗口，等待用户点击 `requirePoints` 个点，返回收集到的坐标数组。

---

# 第二部分：`mouseback.cpp`

## 一、构造函数

```cpp
MouseBack::MouseBack(const std::string& windowName, int requirePoints)
    : windowName(windowName), maxpoints(requirePoints) {}
```

初始化列表设置窗口名和最大点数。

---

## 二、鼠标回调

```cpp
void MouseBack::onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        MouseBack* self = static_cast<MouseBack*>(userdata);
        if (self->points.size() < self->maxpoints)
        {
            self->points.emplace_back(cv::Point2f(x, y));
            std::cout << "已记录点：" << self->points.size()
                      << " (" << x << ", " << y << ")" << std::endl;
        }
    }
}
```

---

### `cv::EVENT_LBUTTONDOWN`

OpenCV 鼠标事件枚举，表示**左键按下**。

其他常用事件：

| 事件 | 含义 |
|------|------|
| `EVENT_LBUTTONDOWN` | 左键按下 |
| `EVENT_RBUTTONDOWN` | 右键按下 |
| `EVENT_MOUSEMOVE` | 鼠标移动 |
| `EVENT_LBUTTONUP` | 左键释放 |

---

### `static_cast<MouseBack*>(userdata)`

OpenCV 的鼠标回调允许传入 `void* userdata`。在 `getPoints` 中，`userdata` 被设为 `this`（当前 `MouseBack` 对象的指针）。

这里通过 `static_cast` 把 `void*` 转回 `MouseBack*`，从而访问对象的成员变量 `points` 和 `maxpoints`。

这是 C/C++ GUI 编程中**回调与对象关联**的经典技巧。

---

### `emplace_back`

在 `points` 末尾直接构造 `cv::Point2f`，避免临时对象拷贝。

---

## 三、`getPoints`

```cpp
std::vector<cv::Point2f> MouseBack::getPoints(const cv::Mat& frame)
{
    points.clear();
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::setMouseCallback(windowName, onMouse, this);
```

---

### `points.clear()`

清空上一轮收集的点，准备重新收集。

---

### `cv::namedWindow`

创建可调整大小的 OpenCV 窗口。

---

### `cv::setMouseCallback`

注册鼠标回调函数。三个参数：

* `windowName`：要监听的窗口名
* `onMouse`：回调函数指针
* `this`：userdata，回调时会原样传回

---

### 显示循环

```cpp
    while (true)
    {
        cv::Mat displayFrame = frame.clone();
```

每轮循环都深拷贝原图，确保绘制不会累积。

---

### 绘制已收集的点

```cpp
        for (size_t i = 0; i < points.size(); ++i) {
            cv::circle(displayFrame, points[i], 10, cv::Scalar(0, 0, 255), -1);
            cv::putText(displayFrame, std::to_string(i+1), points[i] + cv::Point2f(10, 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        }
        // 两点模式下，实时绘制矩形预览
        if (maxpoints == 2 && points.size() == 2) {
            int x1 = static_cast<int>(points[0].x);
            int y1 = static_cast<int>(points[0].y);
            int x2 = static_cast<int>(points[1].x);
            int y2 = static_cast<int>(points[1].y);
            int x = std::min(x1, x2);
            int y = std::min(y1, y2);
            int w = std::abs(x1 - x2);
            int h = std::abs(y1 - y2);
            cv::rectangle(displayFrame, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
        }
```

对每个已收集的点：

1. 画红色实心圆（半径 10）
2. 在点的右下方画绿色数字标签（1, 2, 3...）

`points[i] + cv::Point2f(10, 10)` 把文字稍微偏移，避免和圆点重叠。

---

### 两点模式矩形预览

当 `maxpoints == 2` 且已收集到两个点时，会实时绘制一个绿色矩形框预览。这常用于标定 ROI 区域，用户可以直观地看到两个对角点形成的矩形范围。

---

### 两点模式矩形预览

```cpp
        // 两点模式下，实时绘制矩形预览
        if (maxpoints == 2 && points.size() == 2) {
            int x1 = static_cast<int>(points[0].x);
            int y1 = static_cast<int>(points[0].y);
            int x2 = static_cast<int>(points[1].x);
            int y2 = static_cast<int>(points[1].y);
            int x = std::min(x1, x2);
            int y = std::min(y1, y2);
            int w = std::abs(x1 - x2);
            int h = std::abs(y1 - y2);
            cv::rectangle(displayFrame, cv::Rect(x, y, w, h), cv::Scalar(0, 255, 0), 2);
        }
```

当 `maxpoints == 2`（即 ROI 标定模式）且已经收集到 2 个点时，实时绘制由这两个点构成的矩形框。

* 用 `std::min` 确定左上角，`std::abs` 计算宽高
* 绿色边框（`cv::Scalar(0, 255, 0)`），线宽 2
* 让用户在点击第二个点后、确认前，能直观看到 ROI 范围

这是 `roi_set_node` 标定前哨站 ROI 时的关键交互反馈。

---

### 显示与等待按键

```cpp
        cv::imshow(windowName, displayFrame);
        int key = cv::waitKey(10);
```

`cv::waitKey(10)` 等待 10 毫秒，同时：

* 刷新窗口显示
* 处理鼠标事件（触发 `onMouse`）
* 检测键盘输入

---

### 完成检测

```cpp
        if (points.size() >= maxpoints) {
            std::cout << "标定完成！" << std::endl;
            cv::waitKey(1000);  // 显示 1 秒后退出
            break;
        }
```

收集到足够点数后，等待 1 秒让用户看到最终效果，然后退出循环。

---

### 取消和撤销

```cpp
        if (key == 'q') {
            std::cout << "标定取消！" << std::endl;
            break;
        }
        if (key == ' ') {
            std::cout << "撤销上一点！" << std::endl;
            points.pop_back();
            continue;
        }
```

* `q`：取消标定，提前退出
* `空格`：撤销上一个点（`pop_back`），方便点错了回退

---

### 清理

```cpp
    cv::destroyWindow(windowName);
    return points;
}
```

关闭窗口，返回收集到的点集。

---

# 四、从 MouseBack 层学到的设计要点

## 1. 静态回调 + userdata

C 风格回调无法直接绑定 C++ 成员函数。通过 `static` + `void* userdata` + `static_cast` 的三步组合，实现了面向对象的回调机制。

## 2. 交互式标定的基本模式

```text
显示图像 → 注册鼠标回调 → 循环(waitKey) → 收集坐标 → 返回
```

这是所有"在图上点选"功能的标准模板。

## 3. 撤销机制

`pop_back()` 提供了简单的撤销功能，提升用户体验。
