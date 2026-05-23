# `PipelineTiming.msg` 讲解

## 消息定义

```yaml
std_msgs/Header header
float64 car_ms
float64 armor_ms
float64 cls_ms
float64 outpost_ms
float64 airplane_ms
float64 total_ms
float64 fps
```

---

## 设计意图

`PipelineTiming` 是一个**性能监控消息**，用于实时发布检测流水线各阶段的耗时分解。它不参与算法逻辑，纯粹服务于性能分析和调优。

---

## 字段详解

| 字段 | 类型 | 含义 |
|------|------|------|
| `header` | `std_msgs/Header` | 继承原始图像的时间戳，方便和检测结果做时间对齐 |
| `car_ms` | `float64` | 第一阶段：车辆检测耗时（ms） |
| `armor_ms` | `float64` | 第二阶段：装甲板检测耗时（ms） |
| `cls_ms` | `float64` | 第三阶段：分类耗时（ms） |
| `outpost_ms` | `float64` | 前哨站检测耗时（ms），未启用时为 0 |
| `airplane_ms` | `float64` | 第四阶段：无人机检测耗时（ms），未启用时为 0 |
| `total_ms` | `float64` | 流水线总耗时（ms） |
| `fps` | `float64` | 当前平滑帧率 |

---

## 数据来源

在 `detect_node` 中，时序数据由 `DetectPipeline::getLatestTiming()` 获取：

```cpp
auto timing = pipeline_->getLatestTiming();
auto timing_msg = std::make_unique<tensorrt_detect_msgs::msg::PipelineTiming>();
timing_msg->header = msg->header;
timing_msg->car_ms = timing.car_ms;
timing_msg->armor_ms = timing.armor_ms;
timing_msg->cls_ms = timing.cls_ms;
timing_msg->outpost_ms = timing.outpost_ms;
timing_msg->airplane_ms = timing.airplane_ms;
timing_msg->total_ms = timing.total_ms;
timing_msg->fps = static_cast<double>(fps_);
timing_pub_->publish(std::move(timing_msg));
```

---

## QoS 配置

```cpp
timing_pub_ = this->create_publisher<tensorrt_detect_msgs::msg::PipelineTiming>(
    "/pipeline_timing", rclcpp::QoS(1));
```

使用 `QoS(1)`（Keep Last 1），因为性能数据只关心最新值，旧值无意义。

---

## 话题

* 话题名：`/pipeline_timing`
* 发布者：`detect_node`
* 订阅者：外部监控工具、调试脚本

---

## 使用场景

### 1. 瓶颈定位

```bash
ros2 topic echo /pipeline_timing
```

实时查看各阶段耗时，快速定位哪个阶段是性能瓶颈：

```text
car_ms: 3.2        ← 第一阶段正常
armor_ms: 12.5     ← 第二阶段偏高，可能是 ROI 太多
cls_ms: 0.8        ← 分类正常
airplane_ms: 0.0   ← 未启用
total_ms: 16.5
fps: 60.2
```

### 2. 回归测试

在修改算法或模型后，对比 `/pipeline_timing` 的历史数据，确认性能是否退化。

### 3. 自适应控制

下游节点可以根据 `total_ms` 动态调整行为。例如：当检测耗时过高时，减少图像发布频率。

---

## `total_ms` 与各阶段的关系

`total_ms ≈ car_ms + armor_ms + cls_ms + outpost_ms + airplane_ms`

但由于前哨站和无人机检测可能在独立线程/间隔中执行，`total_ms` 不一定严格等于各阶段之和。它反映的是**主线程从进入到离开流水线**的实际墙钟时间。
