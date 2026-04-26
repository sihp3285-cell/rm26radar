# `ui.hpp` + `ui.cpp` 逐行讲解

这两份文件是 **standalone 模式下的 UI 管理器**，负责把检测图像和小地图水平拼接，显示到一个窗口中，并处理键盘事件。

注意：ROS2 模式下 `display_node` 替代了这个 UI 类，但 standalone 模式下仍然使用它。

---

# 第一部分：`ui.hpp`

## 一、头文件保护

```cpp
#ifndef __UI_HPP__
#define __UI_HPP__
```

标准 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include <string>
#include "radarmap.hpp"
```

OpenCV 用于窗口和图像操作，`radarmap.hpp` 提供 `Mappoint` 结构体（虽然代码里没直接用）。

---

## 三、`UI` 类

```cpp
class UI
{
```

---

### 私有成员

```cpp
private:
    std::string windowName;  // 窗口名称
    cv::Rect btnrect;        // 按钮矩形（预留，未使用）
```

`btnrect` 在当前代码中没有使用，可能是为以后添加按钮交互预留的。

---

### 公共接口

```cpp
public:
    UI(const std::string& windowName);
    int update(const cv::Mat& frame, cv::Mat& radarImg, bool isPaused);
};
```

---

#### `update`

核心接口。每帧调用一次，负责：

1. 缩放小地图到与主图像相同高度
2. 水平拼接
3. 显示
4. 等待键盘输入并返回按键值

---

# 第二部分：`ui.cpp`

## 一、构造函数

```cpp
UI::UI(const std::string& windowName)
    : windowName(windowName)
{
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 1920, 720);
}
```

创建可调整大小的窗口，初始尺寸 1920×720。

这个宽高比（2.67:1）很宽，因为后面要水平拼接两张图：

* 左侧：检测图像（如 1280×720）
* 右侧：小地图（如 640×640 缩放后）

---

## 二、`update`

```cpp
int UI::update(const cv::Mat& frame, cv::Mat& radarImg, bool isPaused)
{
```

---

### 缩放小地图

```cpp
    int targetHeight = frame.rows;
    int targetWidth = static_cast<int>(radarImg.cols * ((float)targetHeight / radarImg.rows));
    cv::Mat finalRadar;
    cv::resize(radarImg, finalRadar, cv::Size(targetWidth, targetHeight));
```

---

#### 保持宽高比缩放

```cpp
scale = targetHeight / radarImg.rows
targetWidth = radarImg.cols * scale
```

小地图缩放到和主图像**相同高度**，宽度按原比例自动计算。

例如：

* 主图像：1280×720
* 小地图：640×640
* `scale = 720 / 640 = 1.125`
* `targetWidth = 640 × 1.125 = 720`
* 缩放后：720×720

---

### 水平拼接

```cpp
    cv::Mat combinedImg;
    cv::hconcat(std::vector<cv::Mat>{frame, finalRadar}, combinedImg);
```

`cv::hconcat`（horizontal concatenate）水平拼接两张图像。

拼接后尺寸：`(1280+720) × 720 = 2000×720`，接近窗口的 1920×720。

---

### 显示

```cpp
    cv::imshow(windowName, combinedImg);
```

---

### 等待键盘输入

```cpp
    return cv::waitKey(isPaused ? 0 : 1);
```

---

#### `isPaused ? 0 : 1`

* `isPaused == true`：`waitKey(0)` **阻塞等待**，直到用户按任意键。视频暂停。
* `isPaused == false`：`waitKey(1)` **非阻塞**，等待 1 毫秒后继续。视频播放。

这是播放器"暂停/播放"功能的核心实现。

---

#### 返回值

返回按键的 ASCII 码。调用方（`standalone_main.cpp`）根据返回值判断用户按了什么键：

* `'q'` 或 `27`（ESC）：退出
* `' '`（空格）：切换暂停状态

---

# 三、从 UI 层学到的设计要点

## 1. 统一高度的拼接

```cpp
cv::resize(radarImg, finalRadar, cv::Size(targetWidth, targetHeight));
```

保证两张图高度一致后再拼接，否则 `hconcat` 会失败或产生黑边。

## 2. 阻塞 vs 非阻塞 `waitKey`

```cpp
waitKey(0);   // 阻塞，用于暂停
waitKey(1);   // 非阻塞，用于播放
```

一个参数控制两种行为，是 OpenCV 视频播放器的标准做法。
