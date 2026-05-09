# `radarmap.hpp` + `radarmap.cpp` 逐行讲解

这两份文件负责**小地图的坐标转换和可视化**。它把 `pose_node` 解算出的世界坐标（米）映射到小地图图像的像素坐标，然后在地图底图上绘制机器人位置。

---

# 第一部分：`radarmap.hpp`

## 一、头文件保护

```cpp
#ifndef RADARMAP_HPP
#define RADARMAP_HPP
```

标准 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
```

OpenCV、字符串、动态数组。

---

## 三、`Mappoint` 结构体

```cpp
struct Mappoint
{
    cv::Point2f map_point;  // 地图像素坐标
    std::string label;      // 显示标签
    int classIdx;           // 类别索引
    int armorColor;         // 装甲颜色 / 队伍 ID
    bool isDead = false;    // 死亡装甲板标志
};
```

---

### 字段解读

| 字段 | 含义 |
|------|------|
| `map_point` | 目标在小地图上的像素坐标 `(x, y)` |
| `label` | 文字标签。实际代码中传空字符串，`drawMap` 内部根据 `classIdx` 和 `isDead` 重新生成 |
| `classIdx` | 类别索引（ARMOR=1, R1=2, R2=3, ..., S=6） |
| `armorColor` | 装甲颜色 / 队伍 ID（UNKNOWN=0, RED=1, BLUE=2） |
| `isDead` | 死亡装甲板标志。`true` 时地图显示黑色圆点和 `"dead"` 标签 |

`Mappoint` 是 `RadarMap` 模块的内部数据结构，连接了 `map_node` 和 `drawMap()`。

> 历史变更：早期版本使用 `teamId` 字段，并通过 `robot_id::getTeamColor/getRobotLabel` 生成颜色和标签。后来还原为原始设计，直接使用 `armorColor` 判断颜色，同时新增 `isDead` 字段专门处理死亡装甲板显示。

---

## 四、`RadarMap` 类

```cpp
class RadarMap
{
```

---

### 私有成员

```cpp
private:
    cv::Mat map;       // 地图底图
    float scale_x;     // X 方向缩放比例
    float scale_y;     // Y 方向缩放比例
    float offset_x;    // X 方向偏移
    float offset_y;    // Y 方向偏移
    bool flip_team_ = false;  // 阵营翻转标志
    bool isCalibrated() const { return m_isCalibrated; }
```

---

#### `map`

地图底图图像。从 `map.png` 加载，通常是场地俯视图。

---

#### `scale_x` / `scale_y`

世界坐标（米）→ 地图像素坐标的缩放比例。

```text
像素坐标 = 世界坐标(米) × scale + offset
```

---

#### `offset_x` / `offset_y`

偏移量。因为世界坐标系的原点通常在场地中心，而地图图像的原点在左上角，所以需要偏移来对齐。

---

#### `m_isCalibrated`

标定完成标志。`calibrate2()` 成功执行后设为 `true`。

---

### 公共接口

```cpp
public:
    bool m_isCalibrated = false;
    RadarMap(const std::string& mapPath, const bool isflip);
    void calibrate2(float race_length, float race_width, int map_width, int map_height);
    cv::Point2f worldtomap(const cv::Point2f& worldPoint) const;
    cv::Mat drawMap(const std::vector<Mappoint>& mappoints,
                    const std::vector<std::string>& classNames) const;
    void setFlipTeam(bool flip) { flip_team_ = flip; }
    bool getFlipTeam() const { return flip_team_; }
```

---

#### `RadarMap`

构造函数。加载地图图片，根据 `isflip` 做旋转。

---

#### `calibrate2`

标定函数。传入场地物理尺寸和地图像素尺寸，计算 `scale` 和 `offset`。

---

#### `worldtomap`

世界坐标 → 地图像素坐标。

---

#### `setFlipTeam` / `getFlipTeam`

运行时动态切换阵营视角：

* `setFlipTeam(true)` → 蓝方视角（底图旋转 180°）
* `setFlipTeam(false)` → 红方视角（不旋转）

> **注意**：`worldtomap()` 本身**不**根据 `flip_team_` 做任何坐标符号翻转。翻转效果完全由 `drawMap()` 中的 `cv::rotate(frame, ..., ROTATE_180)` 实现。这意味着世界坐标到地图坐标的线性映射关系始终不变，视觉上的"翻转"是通过旋转整张底图 180° 来完成的。

---

#### `drawMap`

在地图底图上绘制所有目标点，返回绘制好的图像。

---

# 第二部分：`radarmap.cpp`

## 一、构造函数

```cpp
RadarMap::RadarMap(const std::string& mapPath, const bool isflip)
{
    map = cv::imread(mapPath);
    if(isflip)
    {
        cv::rotate(map, map, cv::ROTATE_90_CLOCKWISE);
    }
    else
    {
        cv::rotate(map, map, cv::ROTATE_90_COUNTERCLOCKWISE);
    }
}
```

---

### `cv::imread`

加载地图图片。OpenCV 默认以 BGR 格式读取。

---

### `cv::rotate`

根据 `isflip` 做 90 度旋转。

这里的设计意图可能是：原始地图图片是横屏或竖屏，需要旋转成与场地坐标系对齐的方向。

* `cv::ROTATE_90_CLOCKWISE`：顺时针 90 度
* `cv::ROTATE_90_COUNTERCLOCKWISE`：逆时针 90 度

注意：这里**不是按 `isflip` 做镜像翻转**，而是做旋转。参数名 `isflip` 可能有点 misleading，实际效果是旋转。

---

## 二、`calibrate2`

```cpp
void RadarMap::calibrate2(float race_length, float race_width,
                          int map_width, int map_height)
{
    if (race_length <= 0 || race_width <= 0 || map_width <= 0 || map_height <= 0) {
        std::cerr << "错误：场地尺寸和地图尺寸必须大于 0" << std::endl;
        return;
    }
```

防御性检查。如果 YAML 配置写错了（比如写 0 或负数），打印错误并返回，避免后续除零。

---

### 使用实际地图尺寸

```cpp
    int actual_map_width = map.cols;
    int actual_map_height = map.rows;
```

不用传入的 `map_width` 和 `map_height`，而是用实际加载的图像尺寸。这样即使 YAML 里的地图尺寸和实际图片不一致，也能正确工作。

传入的 `map_width` 和 `map_height` 在这里实际上**没有被使用**。

---

### 计算缩放比例

```cpp
    scale_x = static_cast<float>(actual_map_width) / race_width;
    scale_y = static_cast<float>(actual_map_height) / race_length;
```

世界坐标（米）→ 像素坐标的缩放比例：

```text
scale_x = 地图像素宽 / 场地物理宽（米）
scale_y = 地图像素高 / 场地物理长（米）
```

例如：场地宽 8 米，地图宽 800 像素，则 `scale_x = 100` 像素/米。

---

### 计算偏移

```cpp
    offset_x = actual_map_width / 2.0f;
    offset_y = actual_map_height / 2.0f;
```

世界坐标系的原点 `(0, 0)` 被映射到地图的**中心**。

这意味着：

* 世界坐标 `(-race_width/2, -race_length/2)` → 地图左上角
* 世界坐标 `(0, 0)` → 地图中心
* 世界坐标 `(race_width/2, race_length/2)` → 地图右下角

---

### 标定完成

```cpp
    m_isCalibrated = true;
}
```

---

## 三、`worldtomap`

```cpp
cv::Point2f RadarMap::worldtomap(const cv::Point2f& worldPoint) const
{
    cv::Point2f mapPoint;
    float wx = worldPoint.x;
    float wy = worldPoint.y;
    mapPoint.x = wx * scale_x + offset_x;
    mapPoint.y = wy * scale_y + offset_y;
    return mapPoint;
}
```

---

### 坐标映射公式

```text
map_x = world_x × scale_x + offset_x
map_y = world_y × scale_y + offset_y
```

这是一个**线性映射**（仿射变换的简化版）：先缩放，再平移。

例如：

* `worldPoint = (4, 7)`（米）
* `scale_x = 100`, `scale_y = 100`（像素/米）
* `offset_x = 400`, `offset_y = 400`（像素）

则：

```text
map_x = 4 × 100 + 400 = 800
map_y = 7 × 100 + 400 = 1100
```

---

## 四、`drawMap`

```cpp
cv::Mat RadarMap::drawMap(const std::vector<Mappoint>& mappoints,
                          const std::vector<std::string>& classNames) const
{
    cv::Mat frame = map.clone();
    if (flip_team_) {
        cv::rotate(frame, frame, cv::ROTATE_180);
    }
```

深拷贝地图底图。这样多次调用 `drawMap` 不会互相污染。

---

### 阵营翻转时的底图旋转

```cpp
if (flip_team_) {
    cv::rotate(frame, frame, cv::ROTATE_180);
}
```

当 `flip_team_ = true` 时，底图本身旋转 180°，确保：

* 地图上的基地、地形等视觉元素与目标点一起"翻转"
* 红方和蓝方的基地位置在视觉上互换
* 文字标签保持正向可读（因为是在旋转后的底图上绘制的）

> 注意：这里**不**存在"坐标翻转"操作。`worldtomap()` 始终返回原始线性映射坐标，`drawMap()` 只是将整个底图旋转 180°。由于底图和目标点一起旋转，视觉上产生了"阵营互换"的效果。如果只做底图旋转而不旋转目标点（或反之），才会出现错位现象。

---

### 遍历每个目标点

```cpp
    for (const auto& mappoint : mappoints)
    {
        cv::Point pt(static_cast<int>(mappoint.map_point.x),
                     static_cast<int>(mappoint.map_point.y));
```

把浮点像素坐标转成整数 `cv::Point`，用于 OpenCV 绘图函数。

---

### 获取绘制颜色

```cpp
        cv::Scalar drawColor;
        if (mappoint.isDead) {
            drawColor = cv::Scalar(0, 0, 0);          // 黑色（死亡）
        } else if (mappoint.armorColor == robot_id::RED) {
            drawColor = cv::Scalar(0, 0, 255);        // 红色
        } else if (mappoint.armorColor == robot_id::BLUE) {
            drawColor = cv::Scalar(255, 0, 0);        // 蓝色
        } else {
            drawColor = cv::Scalar(0, 255, 255);      // 青色（未知）
        }
```

颜色由 `isDead` 和 `armorColor` 共同决定：

* **死亡** → 黑色
* **红色** → `(0, 0, 255)`（BGR 红色）
* **蓝色** → `(255, 0, 0)`（BGR 蓝色）
* **未知** → `(0, 255, 255)`（青色）

---

### 绘制圆点

```cpp
        int baseRadius = 6;
        int strokeSize = 2;
        cv::circle(frame, pt, baseRadius + strokeSize, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);
        cv::circle(frame, pt, baseRadius, drawColor, -1, cv::LINE_AA);
```

画两个同心圆：

1. 外圈：白色，半径 8（`6+2`），作为描边
2. 内圈：队伍颜色（或死亡黑色），半径 6

`-1` 表示填充圆。`cv::LINE_AA` 开启抗锯齿，边缘更平滑。

这种"白边 + 彩色芯"的设计让圆点在各种地图背景上都清晰可见。

---

### 绘制标签

```cpp
        std::string label;
        if (mappoint.isDead) {
            label = "dead";
        } else if (mappoint.classIdx >= 0 && mappoint.classIdx < classNames.size()) {
            label = classNames[mappoint.classIdx];
        }

        if (!label.empty()) {
            cv::Point textPt(pt.x + 10, pt.y - 10);
            cv::putText(frame, label, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        cv::Scalar(255, 255, 255), 5, cv::LINE_AA);
            cv::putText(frame, label, textPt, cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        drawColor, 2, cv::LINE_AA);
        }
    }
    return frame;
}
```

---

#### 标签生成逻辑

标签由 `isDead` 和 `classNames` 共同决定：

* **死亡** → `"dead"`
* **存活** → `classNames[classIdx]`（如 `"R1"`、`"R4"`、`"S"`）

#### 双层文字绘制

和圆点一样，文字也画两层：

1. 底层：白色，线宽 5（粗描边）
2. 上层：队伍颜色（或死亡黑色），线宽 2（实际文字）

这是图像可视化中的**常见技巧**：粗白边 + 细彩字，在任何背景上都有足够的对比度。

---

#### 文字位置

```cpp
cv::Point textPt(pt.x + 10, pt.y - 10);
```

标签放在圆点的右上方，避免和圆点重叠。

---

# 五、从 RadarMap 层学到的设计要点

## 1. 坐标映射的简洁性

```text
像素 = 世界(米) × scale + offset
```

线性映射足够应付大多数俯视图场景。如果场地不是矩形或需要更复杂投影，可以扩展为透视变换（`cv::getPerspectiveTransform` + `cv::warpPerspective`）。

## 2. 深拷贝保护底图

```cpp
cv::Mat frame = map.clone();
```

每次绘制都从原始底图开始，避免多次绘制叠加导致的残留。

## 3. 双层绘制技巧

```cpp
cv::putText(..., white, 5);  // 粗白边
cv::putText(..., color, 2);  // 细彩字
```

不用复杂的外发光算法，用两次绘制实现高对比度标签。
