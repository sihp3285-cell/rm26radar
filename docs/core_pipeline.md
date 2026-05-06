# `pipeline.hpp` + `pipeline.cpp` 逐行讲解

这两份文件是项目的**检测流水线 orchestration 层**。如果说 `model.hpp/cpp` 是"引擎"，那 `pipeline.hpp/cpp` 就是"变速箱"——它决定什么时候用哪个引擎，以及怎么处理引擎的输出。

在你的系统里，`DetectPipeline` 管理三个 `Model` 实例，按**级联（Cascade）**方式执行：

```text
输入图像
    ↓
第一阶段：目标检测（detectModel_）→ 找车辆
    ↓
第二阶段：装甲板检测（armorDetector_）→ 在车辆 ROI 内找装甲板
    ↓
第三阶段：分类（classifyModel_）→ 识别装甲板属于哪个机器人
    ↓
合并结果，返回
```

---

# 第一部分：`pipeline.hpp`

## 一、头文件保护

```cpp
#pragma once
```

和 `#ifndef/#define/#endif` 等价的头文件保护，更简洁，但非标准 C++（几乎所有现代编译器都支持）。

---

## 二、引入的头文件

```cpp
#include "model.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include "ConfigManager.hpp"
```

---

### `"model.hpp"`

需要 `Model` 类和 `Result` 结构体。`DetectPipeline` 内部持有三个 `Model` 对象。

---

### `<opencv2/opencv.hpp>` / `<vector>`

`process()` 接收 `cv::Mat` 输入，返回 `std::vector<Result>`。

---

### `"ConfigManager.hpp"`

`DetectPipeline` 的构造函数接收 `Config&`，从中读取三个模型的路径和参数。

---

## 三、`DetectPipeline` 类

```cpp
class DetectPipeline {
public:
    DetectPipeline(Config& cfg);
    std::vector<Result> process(const cv::Mat& frame);

private:
    Model  detectModel_;      // 第一阶段：目标检测
    Model  armorDetector_;    // 第二阶段：装甲板检测
    Model  classifyModel_;    // 第三阶段：分类
    Config cfg_;              // 配置副本

    std::vector<Result> runDetect(const cv::Mat& frame);
    std::vector<Result> runArmorDetect(const cv::Mat& frame,
                                       const std::vector<Result>& detections);
    void runClassify(const cv::Mat& frame, std::vector<Result>& detections);
};
```

---

### 设计模式：策略封装

`DetectPipeline` 是一个**外观模式（Facade Pattern）**的应用：

> 把三个独立的 `Model` 调用、ROI 裁剪、结果合并等复杂逻辑，封装成一个简单的 `process(frame)` 接口。

外部代码（如 `detect_node`）不需要知道里面有三阶段检测，只需要：

```cpp
auto results = pipeline_->process(frame);
```

---

### `modelType` 辅助函数

```cpp
Model::ModelType modelType(const std::string& modelType)
{
    if(modelType == "DETECT")
        return Model::ModelType::DETECT;
    else if(modelType == "CLASSIFY")
        return Model::ModelType::CLASSIFY;
    else {
        std::cerr << "错误：未知的类型" << std::endl;
        return Model::ModelType::UNKNOWN;
    }
}
```

把配置文件中的字符串（如 `"DETECT"`、`"CLASSIFY"`）转成 `Model::ModelType` 枚举。

这个函数是 `public` 的，但实际上只在构造函数内部使用。它本可以是 `private` 或 `static` 的。

---

### 三个私有方法的分工

| 方法 | 阶段 | 输入 | 输出 |
|------|------|------|------|
| `runDetect` | 一 | 原图 | 车辆检测结果 |
| `runArmorDetect` | 二 | 原图 + 车辆框 | 装甲板检测结果 |
| `runClassify` | 三 | 原图 + 装甲板框 | 类别 ID 更新 |

---

# 第二部分：`pipeline.cpp`

## 一、构造函数

```cpp
DetectPipeline::DetectPipeline(Config& cfg)
    : detectModel_(cfg.model.modelPath, cfg.model.imgSize1,
                   cfg.model.scoreThreshold1, cfg.model.iouThreshold1,
                   cfg.model.isNMS1, modelType(cfg.model.modelType1)),
      armorDetector_(cfg.model.armorModelPath, cfg.model.imgSize2,
                     cfg.model.scoreThreshold2, cfg.model.iouThreshold2,
                     cfg.model.isNMS2, modelType(cfg.model.modelType2)),
      classifyModel_(cfg.model.classifyModelPath, cfg.model.imgSize3,
                     cfg.model.scoreThreshold3, cfg.model.iouThreshold3,
                     cfg.model.isNMS3, modelType(cfg.model.modelType3)),
      cfg_(cfg)
{}
```

---

### 初始化列表（Member Initializer List）

用初始化列表而不是赋值，是 C++ **构造函数的最佳实践**：

* 效率更高：直接构造成员，避免默认构造 + 赋值
* 对于没有默认构造函数的类（如 `Model`），只能用初始化列表
* `const` 成员和引用成员必须在初始化列表中初始化

---

### 三个模型的参数来源

从 `Config::model` 中读取的三个模型配置：

| 模型 | 配置前缀 | 用途 |
|------|---------|------|
| `detectModel_` | `modelPath`, `imgSize1`, `scoreThreshold1`... | 第一阶段目标检测 |
| `armorDetector_` | `armorModelPath`, `imgSize2`, `scoreThreshold2`... | 第二阶段装甲板检测 |
| `classifyModel_` | `classifyModelPath`, `imgSize3`, `scoreThreshold3`... | 第三阶段分类 |

每个模型可以有不同的输入尺寸、阈值、NMS 策略。这种灵活性让你可以：

* 用 YOLOv8 做目标检测（640×640）
* 用轻量 YOLO 做装甲板检测（320×320）
* 用 RepVGG 做分类（224×224）

---

### `cfg_(cfg)`

保存配置对象的**副本**。注意这里是拷贝构造，不是引用。

为什么存副本而不是引用？因为 `Config` 对象可能由外部（如 `detect_node`）管理，如果 `detect_node` 被销毁了，`Config` 引用就会悬空。存副本更安全，虽然有一点点内存开销。

---

## 二、第一阶段：`runDetect`

```cpp
std::vector<Result> DetectPipeline::runDetect(const cv::Mat& frame) {
    detectModel_.Detect(frame);
    cv::Rect roi = detectModel_.roi;
    if(roi.width >= 150 || roi.height >= 150) {
        return std::vector<Result>();
    }
    return detectModel_.detectResults;
}
```

---

### 调用目标检测

```cpp
detectModel_.Detect(frame);
```

对整帧图像跑第一阶段检测，找出所有车辆。

---

### ROI 过滤

```cpp
cv::Rect roi = detectModel_.roi;
if(roi.width >= 150 || roi.height >= 150) {
    return std::vector<Result>();
}
```

这段代码看起来有点奇怪。`detectModel_.roi` 在 `model.hpp` 里是一个公共成员变量，但 `model.cpp` 里似乎没有在 `Detect()` 中设置它。

可能的历史原因：早期版本可能用 `roi` 传递某种全局 ROI 信息。目前这段代码的实际效果取决于 `detectModel_.roi` 的初始值（默认 `cv::Rect()`，即全零）。

`roi.width >= 150 || roi.height >= 150` 对于默认空矩形为 `false`，所以通常不影响流程。

---

### 返回结果

```cpp
return detectModel_.detectResults;
```

返回第一阶段检测到的所有车辆结果。每个 `Result` 的 `box` 是车辆框，`idx` 是车辆类别。

---

## 三、第二阶段：`runArmorDetect`

```cpp
std::vector<Result> DetectPipeline::runArmorDetect(const cv::Mat& frame,
                                                    const std::vector<Result>& detections) {
    std::vector<Result> armorResults;
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);
```

---

### `imgBound`

图像边界矩形。后面做 ROI 裁剪时，用它确保不越界。

---

### 遍历每个车辆检测结果

```cpp
    for (const auto& det : detections) {
        cv::Rect roi = det.box & imgBound;
```

---

### `det.box & imgBound`

`cv::Rect` 的 `&` 运算符表示**矩形交集**。如果车辆检测框有一部分超出了图像边界，取交集后得到安全的 ROI。

例如：车辆框的右下角超出了图像右边界，交集操作会把它裁剪到图像内。

---

### ROI 尺寸过滤

```cpp
        if (roi.width < cfg_.model.minRoiSize || roi.height < cfg_.model.minRoiSize)
            continue;
```

如果 ROI 太小（比如 `minRoiSize = 20`），跳过。太小的 ROI 里不可能有有效的装甲板，直接跳过可以节省推理时间。

---

### 在 ROI 内检测装甲板

```cpp
        if (!armorDetector_.Detect(frame(roi)))
            continue;
```

---

#### `frame(roi)`

OpenCV 的 `cv::Mat` 重载了 `operator()`，支持用 `cv::Rect` 做**浅拷贝 ROI 提取**。

```cpp
cv::Mat roiMat = frame(roi);
```

这行操作**不复制像素数据**，只是创建一个新的 `cv::Mat` 头，指向原图 `frame` 中 `roi` 对应区域的内存。时间复杂度 O(1)。

这是计算机视觉中**区域裁剪**的标准高效做法。

---

#### `armorDetector_.Detect(roiMat)`

对裁剪后的 ROI 跑第二阶段检测。模型输入是 ROI 小图，输出是 ROI 坐标系内的装甲板检测框。

---

### 只保留置信度最高的装甲板

```cpp
        if (!armorDetector_.detectResults.empty()) {
            auto maxArmor = std::max_element(armorDetector_.detectResults.begin(),
                                            armorDetector_.detectResults.end(),
                [](const Result& a, const Result& b) {
                    return a.confidence < b.confidence;
                });
```

---

#### `std::max_element`

C++ 标准库算法，在范围内找出**最大元素**的迭代器。

这里用 lambda 函数作为比较器：

```cpp
[](const Result& a, const Result& b) { return a.confidence < b.confidence; }
```

表示"按 confidence 从小到大排，找最大的"。

---

#### 为什么只保留一个装甲板？

在 RoboMaster 场景中，一辆车通常只有 **1~4 个装甲板**（前后左右），但在某个时刻正对相机的通常只有 **1 个**（最多 2 个）。

如果保留所有装甲板检测结果，后续分类和处理会变得复杂。只保留置信度最高的一个，是一种**简化假设**，在实践中效果通常足够好。

如果要支持多装甲板检测，可以去掉这行限制。

---

### 坐标转换与结果填充

```cpp
            Result armor = *maxArmor;
            int raw_id = armor.idx;

            armor.box.x += roi.x;
            armor.box.y += roi.y;
            armor.car_box = det.box;

            armor.idx = 1;
            armor.armorColor = raw_id;

            armor.worldPoint = cv::Point2f(det.box.x + det.box.width  / 2.0f,
                                           det.box.y + det.box.height / 2.0f);
            armorResults.push_back(armor);
```

---

#### 坐标系回变换

```cpp
armor.box.x += roi.x;
armor.box.y += roi.y;
```

`armorDetector_` 的输出坐标是在 **ROI 局部坐标系**内的（以 ROI 左上角为原点）。需要加上 `roi.x` 和 `roi.y`，转回**原图全局坐标系**。

这是级联检测中**最常见的坐标 bug 来源**。如果忘了这一步，装甲板框会显示在图像左上角附近。

---

#### `car_box`

```cpp
armor.car_box = det.box;
```

把车辆框（第一阶段的检测结果）保存到装甲板结果的 `car_box` 字段。

这个字段在后续 `pose_node` 中非常重要：`pose_node` 优先用 `car_box` 的底部中心做世界坐标解算，因为车辆底部中心比装甲板中心更稳定。

---

#### `idx`、`armorColor` 和 `isDead`

```cpp
constexpr int DEAD_ARMOR_ID = 0;  // 装甲板检测模型中死亡装甲板的原始类别 ID

armor.idx = robot_id::ARMOR;      // 固定为 ARMOR 类别（1）
armor.isDead = (raw_id == DEAD_ARMOR_ID);
if (armor.isDead) {
    armor.armorColor = robot_id::UNKNOWN;  // 死亡装甲板颜色标记为未知
} else {
    armor.armorColor = raw_id;             // 存活装甲板保留原始颜色/队伍 ID
}
```

这里做了**关键的语义分离**：

* `idx` 是类别索引。第二阶段模型只检测装甲板，所以统一设为 `robot_id::ARMOR`（1）
* `isDead` 是**独立的死亡状态标志**。如果二层模型输出 `raw_id == 0`，表示检测到的是**死亡装甲板**
* `armorColor` 现在**只表示颜色/队伍**（红/蓝），不再和死亡状态混用。死亡装甲板的颜色被显式设为 `UNKNOWN`（0）

> 为什么要分离 `isDead` 和 `armorColor`？因为原始代码把 `armorColor == 0` 同时当作"未知颜色"和"死亡装甲板"两种语义，导致下游节点（如地图绘制）无法区分。现在死亡状态由独立的 `bool isDead` 承载，逻辑更清晰。

---

#### `worldPoint`

```cpp
armor.worldPoint = cv::Point2f(det.box.x + det.box.width / 2.0f,
                               det.box.y + det.box.height / 2.0f);
```

这里计算的 "worldPoint" 实际上只是**车辆框的中心像素坐标**，并不是真正的世界坐标。

真正的世界坐标解算在 `pose_node` 中通过 `PoseSolver` 完成。这里的 `worldPoint` 更像是一个**中间传递值**，可能是历史遗留或给 standalone 模式用的快速估算。

---

## 四、第三阶段：`runClassify`

```cpp
void DetectPipeline::runClassify(const cv::Mat& frame, std::vector<Result>& detections) {
    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);

    for (auto& armor : detections) {
        if (armor.isDead) {
            armor.idx = robot_id::ARMOR;
            continue;
        }

        cv::Rect safeBox = armor.box & imgBound;
        if (safeBox.width <= 0 || safeBox.height <= 0) {
            continue;
        }

        cv::Mat armorROI = frame(safeBox);

        int raw_id = classifyModel_.predictClass(armorROI);
        if (raw_id == 4) {
            armor.idx = 6;  // S = 哨兵
        } else if (raw_id >= 0 && raw_id <= 3) {
            armor.idx = raw_id + 2;  // R1=2, R2=3, R3=4, R4=5
        }
    }
}
```

---

### 遍历每个装甲板

```cpp
for (auto& armor : detections) {
```

注意这里用 `auto&`（引用），因为需要修改 `armor.idx`。

---

### 死亡装甲板过滤

```cpp
        if (armor.isDead) {
            armor.idx = robot_id::ARMOR;
            continue;
        }
```

如果 `armor.isDead == true`，表示这是**死亡装甲板**，不进入第三层分类模型，直接保持 `idx = robot_id::ARMOR`（1）。

这与原始代码的行为一致——死亡装甲板在第二层结束，不消耗分类模型的推理资源。但判断条件从 `armorColor == 0` 改成了显式的 `isDead`，避免了语义混淆。

---

### 安全裁剪

```cpp
        cv::Rect safeBox = armor.box & imgBound;
        if (safeBox.width <= 0 || safeBox.height <= 0) {
            continue;
        }
```

再次用交集确保不越界。如果装甲板框完全在图像外（`width <= 0` 或 `height <= 0`），跳过。

---

### 分类推理

```cpp
        cv::Mat armorROI = frame(safeBox);
        int raw_id = classifyModel_.predictClass(armorROI);
```

提取装甲板 ROI，跑分类模型。`predictClass` 返回分类模型预测的原始类别 ID。

---

### 类别 ID 映射

```cpp
        if (raw_id == 4) {
            armor.idx = 6;           // 哨兵
        } else if (raw_id >= 0 && raw_id <= 3) {
            armor.idx = raw_id + 2;  // raw_id 0→R1(2), 1→R2(3), 2→R3(4), 3→R4(5)
        }
```

---

#### 映射表

| raw_id（分类模型输出） | idx（项目标准） | 含义 |
|----------------------|---------------|------|
| 0 | 2 | R1（1号机器人） |
| 1 | 3 | R2（2号机器人） |
| 2 | 4 | R3（3号机器人） |
| 3 | 5 | R4（4号机器人） |
| 4 | 6 | S（哨兵） |

分类模型的输出空间是 5 类（0~4），但项目标准类别 ID 是 2~6（参考 `robot_id.hpp`）。所以需要做 +2 偏移映射。

---

## 五、前哨站检测：`runOutpostDetect`

```cpp
std::vector<Result> DetectPipeline::runOutpostDetect(const cv::Mat& frame) {
    std::vector<Result> results;
    if (!cfg_.model.outpostEnabled || cfg_.model.outpostRoi.size() != 4) {
        return results;
    }

    const cv::Rect imgBound(0, 0, frame.cols, frame.rows);
    cv::Rect outpostRoi(
        cfg_.model.outpostRoi[0],
        cfg_.model.outpostRoi[1],
        cfg_.model.outpostRoi[2],
        cfg_.model.outpostRoi[3]
    );
    cv::Rect safeRoi = outpostRoi & imgBound;
    if (safeRoi.width <= 0 || safeRoi.height <= 0) {
        return results;
    }

    bool hasValidDetection = false;
    Result bestResult;
    float bestConf = -1.0f;

    if (armorDetector_.Detect(frame(safeRoi))) {
        for (auto& res : armorDetector_.detectResults) {
            if (res.confidence < cfg_.model.outpostScoreThreshold) {
                continue;
            }
            if (res.confidence > bestConf) {
                bestConf = res.confidence;
                bestResult = res;
                hasValidDetection = true;
            }
        }
    }

    if (hasValidDetection) {
        outpostMissCount_ = 0;
        outpostIsDead_ = false;

        bestResult.box.x += safeRoi.x;
        bestResult.box.y += safeRoi.y;
        bestResult.idx = robot_id::OUTPOST;
        bestResult.car_box = safeRoi;
        bestResult.isDead = false;
        outpostLastBox_ = bestResult.box;
        results.push_back(bestResult);
    } else {
        outpostMissCount_++;
        if (outpostMissCount_ >= cfg_.model.outpostMissThreshold) {
            outpostMissCount_ = cfg_.model.outpostMissThreshold;
            outpostIsDead_ = true;

            Result deadResult;
            deadResult.idx = robot_id::OUTPOST;
            deadResult.isDead = true;
            deadResult.confidence = 0.0f;
            if (outpostLastBox_.width > 0 && outpostLastBox_.height > 0) {
                deadResult.box = outpostLastBox_;
            } else {
                deadResult.box = safeRoi;
            }
            deadResult.car_box = safeRoi;
            results.push_back(deadResult);
        }
    }
    return results;
}
```

---

### 独立 ROI 截取

前哨站检测不依赖第一阶段的目标检测结果，而是直接使用配置文件中的固定 `outpostRoi`。这是因为前哨站在场地中的位置相对固定，手动标定 ROI 比让模型在全图搜索更稳定、更快。

### 复用 armor 模型

```cpp
armorDetector_.Detect(frame(safeRoi))
```

前哨站的"装甲板"用和第二阶段相同的 `armorDetector_` 检测，不单独加载模型，节省显存和初始化时间。

### 置信度过滤

```cpp
if (res.confidence < cfg_.model.outpostScoreThreshold) continue;
```

前哨站有独立的置信度阈值 `outpostScoreThreshold`，和第二阶段的全局 `scoreThreshold2` 解耦，方便针对前哨站场景单独调优。

### 帧间累计计数器（核心机制）

```cpp
if (hasValidDetection) {
    outpostMissCount_ = 0;      // 检测到 → 清零
    outpostIsDead_ = false;
} else {
    outpostMissCount_++;        // 未检测到 → 累加
    if (outpostMissCount_ >= cfg_.model.outpostMissThreshold) {
        outpostIsDead_ = true;  // 达到阈值 → 判定死亡
    }
}
```

这是前哨站检测的**关键稳定性设计**：

* **检测到**：计数器清零，状态 = 存活
* **未检测到**：计数器累加
* **达到阈值**（默认 20 帧）：状态 = 摧毁（死亡）

避免偶尔一帧漏检就导致状态跳变，只有**持续漏检**才判定前哨站被摧毁。

### 死亡状态输出

即使前哨站被判定死亡，仍然会输出一个 `isDead = true` 的结果。这样下游节点（如 QT 前端）能持续显示"前哨站：摧毁"的状态，而不是直接消失。

---

## 六、总入口：`process`

```cpp
std::vector<Result> DetectPipeline::process(const cv::Mat& frame) {
    auto cars = runDetect(frame);
    auto armors = runArmorDetect(frame, cars);
    runClassify(frame, armors);

    std::vector<Result> all;
    all.insert(all.end(), cars.begin(), cars.end());
    all.insert(all.end(), armors.begin(), armors.end());

    auto outposts = runOutpostDetect(frame);
    all.insert(all.end(), outposts.begin(), outposts.end());

    return all;
}
```

---

### 四阶段调用

```text
1. runDetect(frame)              → cars（车辆检测结果）
2. runArmorDetect(frame, cars)   → armors（装甲板检测结果）
3. runClassify(frame, armors)    → 更新 armors 的 idx
4. runOutpostDetect(frame)       → outposts（前哨站检测结果）
```

---

### 结果合并

```cpp
all.insert(all.end(), outposts.begin(), outposts.end());
```

前哨站结果独立追加到最终列表。返回的 `vector<Result>` 里包含：

* 车辆检测框（`idx = 0`，即 `CAR`）
* 装甲板检测框（`idx = 2~6`，即 `R1~R4`、`S`）
* 前哨站（`idx = 7`，即 `OUTPOST`）

---

# 七、从 Pipeline 层学到的设计要点

## 1. 级联检测（Cascade Detection）

把一个复杂问题拆成三个简单子问题：

```text
找车辆 → 找装甲板 → 识别身份
```

每个子问题用一个专门的轻量模型解决，比一个超大模型端到端解决更灵活、更容易优化。

## 2. ROI 级联裁剪

```cpp
frame(roi)  // 浅拷贝，O(1)
```

第二阶段和第三阶段都在 ROI 上运行，输入图像尺寸远小于原图，推理速度更快。

## 3. 坐标系管理

级联检测最容易出错的点就是坐标系转换：

```text
原图 → ROI 局部坐标 → 原图全局坐标
     ↑_________________↓（别忘了加回 roi.x/y）
```

## 4. 置信度筛选

只保留置信度最高的装甲板，是一种**业务假设驱动的简化**。如果你的场景需要同时检测多个装甲板，可以修改这里的逻辑。

## 5. 外观模式（Facade）

```cpp
std::vector<Result> process(const cv::Mat& frame);
```

一个简单接口隐藏了三阶段检测的复杂细节。`detect_node` 只需要调用这一个函数，不需要知道里面发生了什么。
