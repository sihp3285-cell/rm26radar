# `map_analyzer.hpp` + `map_analyzer.cpp` 逐行讲解

这两份文件实现了 **战术态势分析器**（Map Analyzer），用于从世界坐标目标中提取战术层面的信息：工程上岛、攻防态势、堡垒威胁等。

在系统中的位置：

```text
pose_node
   ↓  /world_targets
map_node
   ├── RadarMap（世界坐标→地图像素坐标，绘图）
   ├── MapAnalyzer（战术态势分析）  ← 这里
   ├── MapTactics 消息发布
   ↓  /map_image, /radar_map, /map_tactics
qt_display_node / 决策节点
```

---

# 第一部分：`map_analyzer.hpp`

## 一、头文件保护与依赖

```cpp
#ifndef MAP_ANALYZER_HPP
#define MAP_ANALYZER_HPP

#include <vector>
#include "tensorrt_detect_msgs/msg/world_target.hpp"
#include "robot_id.hpp"
```

依赖 `WorldTarget` 消息类型和 `robot_id` 枚举定义。

---

## 二、`MapAnalyzer` 类

```cpp
class MapAnalyzer {
public:
    explicit MapAnalyzer(int our_team_id = robot_id::BLUE);
    void evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets);
    void setFieldXFlip(bool flip) { field_x_flip_ = flip; }
    void setTeamByFlip(bool flip_team);

    int engineer_on_island() const { return engineer_on_island_; }
    int opponent_attack() const { return opponent_attack_; }
    int our_attack() const { return our_attack_; }
    int opponent_near_fortress() const { return opponent_near_fortress_; }

private:
    int our_team_id_ = robot_id::BLUE;
    int my_team_ = robot_id::BLUE;
    int opponent_team_ = robot_id::RED;
    bool field_x_flip_ = false;

    int engineer_on_island_ = 0;
    int opponent_attack_ = 0;
    int our_attack_ = 0;
    int opponent_near_fortress_ = 0;

    std::pair<float, float> toFieldCoord(float world_x, float world_z) const;
};
```

---

### 公共接口

| 方法 | 作用 |
|------|------|
| `evaluate()` | 分析一帧 `WorldTargetArray` 中的所有目标，更新战术状态 |
| `setTeamByFlip()` | 根据阵营翻转状态更新我方/对方阵营 |
| `setFieldXFlip()` | 设置场地 X 轴翻转（适配不同标定方向） |
| `engineer_on_island()` | 返回工程上岛状态：0=无, 1=我方上岛, 2=对方上岛 |
| `opponent_attack()` | 返回对方是否发起大攻：0=否, 1=是 |
| `our_attack()` | 返回我方是否发起大攻：0=否, 1=是 |
| `opponent_near_fortress()` | 返回对方是否接近我方堡垒：0=否, 1=是 |

---

### `setTeamByFlip` 方法

```cpp
void setTeamByFlip(bool flip_team) {
    if (flip_team) {
        my_team_ = robot_id::RED;
        opponent_team_ = robot_id::BLUE;
    } else {
        my_team_ = robot_id::BLUE;
        opponent_team_ = robot_id::RED;
    }
}
```

当 `qt_display_node` 切换红蓝方视角时，`map_node` 收到 `/flip_team` 话题后调用此方法，同步更新分析器的阵营归属。

---

# 第二部分：`map_analyzer.cpp`

## 一、构造函数

```cpp
MapAnalyzer::MapAnalyzer(int our_team_id)
    : our_team_id_(our_team_id),
      my_team_(our_team_id),
      opponent_team_((our_team_id == robot_id::RED) ? robot_id::BLUE : robot_id::RED)
{}
```

---

### `our_team_id` 参数

由 `map_node` 的 `out_team_id` 参数决定（默认 RED）。这个值表示"我们实际是哪一方"，不受界面翻转影响。

---

## 二、坐标转换：`toFieldCoord`

```cpp
std::pair<float, float> MapAnalyzer::toFieldCoord(float world_x, float world_z) const
{
    float field_x = field_x_flip_ ? (world_z + 14.0f) : (14.0f - world_z);
    float field_y = world_x + 7.5f;
    return {field_x, field_y};
}
```

---

### 为什么要坐标转换？

`PoseSolver` 输出的 `world_x` 和 `world_z` 是**相机坐标系**下的场地坐标，正方向取决于标定时相机的朝向。

`MapAnalyzer` 需要的是**场地坐标系**下的坐标（field_x: 0~28m 沿长边, field_y: 0~15m 沿短边），才能判断"距离我方基地多远"、"是否在岛屿区域"等。

---

### `field_x_flip_` 的作用

* `false`：`world_z` 正方向指向蓝方 → `field_x = 14 - world_z`（红方在 0，蓝方在 28）
* `true`：`world_z` 正方向指向红方 → `field_x = world_z + 14`（红方在 0，蓝方在 28）

`map_node` 中设置 `analyzer_->setFieldXFlip(!flip_team_)`，保证无论界面怎么翻转，`field_x` 始终以红方基地为 0、蓝方基地为 28。

---

### 偏移量

* `+14.0f`：场地半长（28m / 2）
* `+7.5f`：场地半宽（15m / 2）

使坐标原点从场地中心移到红方基地角落。

---

## 三、核心分析方法：`evaluate`

```cpp
void MapAnalyzer::evaluate(const std::vector<tensorrt_detect_msgs::msg::WorldTarget>& targets)
{
    engineer_on_island_ = 0;
    opponent_attack_ = 0;
    our_attack_ = 0;
    opponent_near_fortress_ = 0;
    int opponent_attack_count = 0;
    int our_attack_count = 0;
    bool our_engineer_on_island = false;
    bool engineer_on_opponent_island = false;
    bool opponent_near_fortress = false;
```

每次调用先清零所有战术标志，然后遍历所有目标重新评估。

---

### 3.1 工程上岛检测

```cpp
if (target.class_id == robot_id::R2) {
    if (target.team_id == my_team_ && dist_from_our >= 10.0f && dist_from_our <= 16.0f &&
        fy >= 6.0f && fy <= 9.0f) {
        our_engineer_on_island = true;
    } else if (target.team_id == opponent_team_ && dist_from_opponent >= 10.0f && dist_from_opponent <= 16.0f &&
               fy >= 6.0f && fy <= 9.0f) {
        engineer_on_opponent_island = true;
    }
}
```

---

#### 判断条件

| 条件 | 含义 |
|------|------|
| `class_id == R2` | 只检查工程机器人 |
| `dist_from_our >= 10 && dist_from_our <= 16` | 距离我方基地 10~16 米（中间区域） |
| `fy >= 6.0f && fy <= 9.0f` | 在场地宽度方向的中间区域（岛屿所在位置） |

---

#### 工程上岛状态编码

| `engineer_on_island_` | 含义 |
|----------------------|------|
| 0 | 无工程上岛 |
| 1 | 我方工程上岛 |
| 2 | 对方工程上岛 |

---

### 3.2 攻防态势检测

```cpp
// 对方大攻
if (target.team_id == opponent_team_ &&
    (target.class_id == robot_id::R1 || target.class_id == robot_id::R3 ||
     target.class_id == robot_id::R4 || target.class_id == robot_id::S)) {
    if (dist_from_opponent > 18.0f) {
        opponent_attack_count++;
    }
}

// 我方大攻
if (target.team_id == my_team_ &&
    (target.class_id == robot_id::R1 || target.class_id == robot_id::R3 ||
     target.class_id == robot_id::R4 || target.class_id == robot_id::S)) {
    if (dist_from_our > 14.0f) {
        our_attack_count++;
    }
}
```

---

#### 大攻判定逻辑

* **对方大攻**：对方有 ≥ 2 个战斗机器人（排除工程 R2）出现在距对方基地 18 米以外的位置（即深入我方半场）
* **我方大攻**：我方有 ≥ 2 个战斗机器人出现在距我方基地 14 米以外的位置（即深入对方半场）

```cpp
if (opponent_attack_count >= 2) {
    opponent_attack_ = 1;
}
if (our_attack_count >= 2) {
    our_attack_ = 1;
}
```

---

#### 为什么阈值不同？

* 对方大攻用 18m：对方需要更深入才算"大攻"（保守判断）
* 我方大攻用 14m：我方深入过半就算"大攻"（激进判断）

这是经验值，可根据实际比赛策略调整。

---

### 3.3 堡垒威胁检测

```cpp
float our_fortress_x = (my_team_ == robot_id::RED) ? 6.0f : 22.0f;
float our_fortress_y = 7.5f;

float dx = fx - our_fortress_x;
float dy = fy - our_fortress_y;
float dist2 = dx * dx + dy * dy;

float radius = 1.8f;

if (dist2 <= radius * radius) {
    opponent_near_fortress = true;
}
```

---

#### 判断逻辑

* 堡垒位置：红方 (6.0, 7.5)、蓝方 (22.0, 7.5)
* 威胁半径：1.8 米
* 只检查对方战斗机器人（排除工程 R2）

如果对方有战斗机器人出现在我方堡垒 1.8 米范围内，触发堡垒威胁警报。

---

## 四、输出状态汇总

```cpp
if (our_engineer_on_island) {
    engineer_on_island_ = 1;
} else if (engineer_on_opponent_island) {
    engineer_on_island_ = 2;
}

if (opponent_attack_count >= 2) {
    opponent_attack_ = 1;
}
if (our_attack_count >= 2) {
    our_attack_ = 1;
}
if (opponent_near_fortress) {
    opponent_near_fortress_ = 1;
}
```

---

# 第三部分：从 MapAnalyzer 学到的设计要点

## 1. 战术分析与渲染分离

`MapAnalyzer` 只做**数值计算**，不碰任何 OpenCV 绘图。它的输出是纯整数标志，`map_node` 负责把这些标志转换为日志、消息或图像。

这种分离让 `MapAnalyzer` 可以被独立测试：手写一组 `WorldTarget`，调用 `evaluate()`，验证输出是否符合预期。

## 2. 阵营感知

`MapAnalyzer` 通过 `setTeamByFlip()` 和 `setFieldXFlip()` 跟踪当前阵营视角。同一组世界坐标，在红方视角和蓝方视角下，"我方"和"对方"的判断完全相反。

## 3. 基于规则的战术判断

当前的战术分析完全基于**硬编码规则**（距离阈值、数量阈值）。这种方式的优点是：

* 逻辑透明，容易调试
* 没有训练数据依赖
* 响应速度快（纯整数/浮点运算）

缺点是阈值需要人工调优，且无法处理更复杂的战术模式（如围攻、佯攻）。未来可以考虑引入轻量级机器学习模型。

## 4. 信息密度优化

`MapTactics` 消息只有 4 个整数字段（`engineer_on_island`, `opponent_attack`, `our_attack`, `opponent_near_fortress`），但在决策层面信息量很大：

* 工程上岛 → 可能要调整资源分配
* 对方大攻 → 需要加强防守
* 我方大攻 → 可以继续推进
* 堡垒威胁 → 需要紧急回防

这 4 个标志浓缩了数十个世界坐标目标的战术含义。
