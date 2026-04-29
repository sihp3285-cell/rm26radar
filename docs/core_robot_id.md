# `robot_id.hpp` 逐行讲解

这是一份**纯头文件工具库**，没有对应的 `.cpp` 文件。它定义了项目中的所有语义常量、枚举和辅助函数，是连接模型输出、消息传递和可视化的"词汇表"。

---

## 一、头文件保护

```cpp
#pragma once
```

简洁的 include guard。

---

## 二、引入的头文件

```cpp
#include <opencv2/opencv.hpp>
#include <string>
```

需要 `cv::Scalar`（颜色）和 `std::string`。

---

## 三、命名空间

```cpp
namespace robot_id {
```

把所有定义放在 `robot_id` 命名空间中，避免全局命名空间污染。

---

## 四、`TeamId` 枚举

```cpp
enum TeamId {
    UNKNOWN = 0,  // 未知/无效
    RED     = 1,  // 红方
    BLUE    = 2,  // 蓝方
};
```

---

### 为什么从 0 开始？

* `0` 作为"无效/未知"是 C/C++ 的惯例
* 在 ROS2 消息中，如果字段没有正确填充，默认值就是 0，自然表示 UNKNOWN
* 避免了未初始化变量导致错误识别为某一方

---

### 与 `armor_color` 的对应

代码注释明确说明：`TeamId` 与 `armor_color` 对齐。

在 `DetectionBox` 和 `Result` 中，`armorColor` 字段的值直接对应这个枚举：

```cpp
// 存活装甲板：armorColor 表示实际颜色/队伍
armor.armorColor = raw_id;  // 1/2 对应 RED/BLUE

// 死亡装甲板：armorColor 显式设为 UNKNOWN，但死亡状态由 isDead 字段独立承载
armor.armorColor = robot_id::UNKNOWN;
armor.isDead = true;
```

> **重要变更**：原始代码中 `armorColor == 0`（UNKNOWN）被同时用来表示"未知颜色"和"死亡装甲板"，导致语义混淆。现在死亡状态由独立的 `bool isDead` 字段承载，`armorColor` 只表示颜色/队伍信息。

---

## 五、`ClassId` 枚举

```cpp
enum ClassId {
    CAR   = 0,  // 车辆
    ARMOR = 1,  // 未分类装甲板
    R1    = 2,  // 1号机器人
    R2    = 3,  // 2号机器人
    R3    = 4,  // 3号机器人
    R4    = 5,  // 4号机器人
    S     = 6,  // 哨兵
};
```

---

### 设计含义

| ID | 含义 | 使用场景 |
|----|------|---------|
| 0 | CAR | 第一阶段目标检测输出 |
| 1 | ARMOR | 第二阶段装甲板检测输出（未分类时） |
| 2~5 | R1~R4 | 分类后的机器人编号 |
| 6 | S | 分类后的哨兵 |

---

### 为什么 CAR = 0？

* 很多检测模型把"背景类"设为 0，但这里 CAR 是前景目标
* 和 `TeamId` 一样，0 作为"默认/基础"类别
* `map_node` 中会 `continue` 跳过 `CAR`，因为小地图不需要显示未分类车辆

---

## 六、`getTeamName`

```cpp
inline std::string getTeamName(int team_id) {
    switch (team_id) {
        case RED:  return "red";
        case BLUE: return "blue";
        default:   return "?";
    }
}
```

---

### `inline`

头文件中定义的函数必须用 `inline`，否则多个 `.cpp` 文件包含这个头文件时，会发生**重复定义链接错误**。

`inline` 告诉编译器：这个函数可以在多个翻译单元中定义，链接时合并为同一个。

---

### 返回值

* `RED` → `"red"`
* `BLUE` → `"blue"`
* 其他 → `"?"`

---

## 七、`getRobotNumber`

```cpp
inline std::string getRobotNumber(int class_id) {
    switch (class_id) {
        case R1: return "1";
        case R2: return "2";
        case R3: return "3";
        case R4: return "4";
        case S:  return "S";
        default: return "";
    }
}
```

把 `class_id` 转成人类可读的编号字符串。

* `R1~R4` → `"1"~"4"`
* `S` → `"S"`
* 其他（CAR、ARMOR）→ `""`

---

## 八、`getRobotLabel`

```cpp
inline std::string getRobotLabel(int team_id, int class_id) {
    if (team_id == UNKNOWN) {
        return "?";
    }
    std::string num = getRobotNumber(class_id);
    if (num.empty()) {
        return "?";
    }
    return getTeamName(team_id) + num;
}
```

---

### 组合标签

把队伍名和机器人编号组合起来：

```text
RED + "3" → "red3"
BLUE + "S" → "blueS"
```

---

### 防御性检查

* `team_id == UNKNOWN` → `"?"`
* `num.empty()`（即 CAR 或 ARMOR）→ `"?"`

确保只有完整的 `(team_id, class_id)` 组合才能生成有效标签。

---

## 九、`getTeamColor`

```cpp
inline cv::Scalar getTeamColor(int team_id) {
    switch (team_id) {
        case RED:  return cv::Scalar(0, 0, 255);    // 红色
        case BLUE: return cv::Scalar(255, 0, 0);    // 蓝色
        default:   return cv::Scalar(0, 255, 255);  // 黄色（未知）
    }
}
```

---

### BGR 格式

OpenCV 默认使用 **BGR**（蓝-绿-红）而不是 RGB：

| 颜色 | B | G | R |
|------|---|---|---|
| 红色 | 0 | 0 | 255 |
| 蓝色 | 255 | 0 | 0 |
| 黄色 | 0 | 255 | 255 |

---

### 使用场景

* `radarmap.cpp` 的 `drawMap()` 中绘制圆点和文字颜色
* `draw.cpp` 的 `drawDetect()` 中绘制检测框颜色

---

## 十、从 robot_id 层学到的设计要点

## 1. 语义常量集中管理

所有 ID 定义在一个地方，避免魔法数字（Magic Number）散布在代码各处。

不好的做法：

```cpp
if (armor.armorColor == 2) { ... }  // 2 是什么？
```

好的做法：

```cpp
if (armor.armorColor == robot_id::BLUE) { ... }
```

## 2. 枚举 + 工具函数的组合

枚举定义常量，工具函数提供人类可读的表示。两者配合，既保证类型安全，又方便调试和显示。

## 3. 头文件-only 库

没有 `.cpp` 文件，所有函数都是 `inline`。包含即可用，无需链接。
