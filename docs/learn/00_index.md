# TensorRT Detect 从零学习指南 —— 总目录

> 本系列文档面向希望从零开始掌握 `tensorrt10_detect` 项目所涉及技术的开发者。项目是一个基于 **ROS2 + C++ + CUDA + TensorRT** 的实时目标检测与位姿解算系统，广泛应用于机器人竞赛视觉领域。

---

## 📚 文档结构

| 编号 | 文档 | 内容概要 |
|:---:|:---|:---|
| 01 | [ROS2 基础](./01_ros2基础.md) | ROS2 核心概念、工作空间、节点、话题、服务、参数 |
| 02 | [ROS2 C++ 节点开发](./02_ros2节点开发.md) | 继承 `rclcpp::Node`、发布订阅、服务、回调、QoS |
| 03 | [ROS2 消息与接口](./03_ros2消息与接口.md) | 自定义 `.msg`、接口生成、`cv_bridge` 图像转换 |
| 04 | [ROS2 Launch 与组件化](./04_ros2_launch与组件.md) | Python Launch、ComposableNode、零拷贝、Executor |
| 05 | [C++ 现代特性](./05_c++现代特性.md) | C++17、智能指针、Lambda、线程同步、原子操作 |
| 06 | [CUDA 与 TensorRT](./06_cuda与tensorrt.md) | CUDA Kernel、Stream、TensorRT10 API、推理流水线 |
| 07 | [OpenCV 与计算机视觉](./07_opencv与计算机视觉.md) | 图像处理、相机标定、PnP、位姿解算、Raycasting |
| 08 | [目标跟踪与卡尔曼滤波](./08_目标跟踪与卡尔曼滤波.md) | 匈牙利算法、Kalman Filter、固定槽位 Tracker |
| 09 | [YAML 配置管理](./09_yaml配置管理.md) | `yaml-cpp`、配置结构体、动态重载、参数验证 |
| 10 | [Qt5 可视化开发](./10_qt5可视化开发.md) | Qt Widgets、信号槽、多线程 UI 更新、ROS2+Qt 融合 |
| 11 | [Eigen 与数学计算](./11_eigen与数学计算.md) | 固定大小矩阵、Kalman 矩阵运算、对齐宏 |
| 12 | [Open3D 与射线检测](./12_open3d与射线检测.md) | 3D 网格加载、`RaycastingScene`、像素到世界坐标 |
| 13 | [CMake 构建系统](./13_cmake构建系统.md) | `ament_cmake`、CUDA 项目配置、Component 注册 |
| 14 | [项目架构与流水线](./14_项目架构与流水线.md) | 节点拓扑、数据流、DetectPipeline、多模型级联 |
| 15 | [从零部署项目](./15_从零部署项目.md) | 环境安装、依赖编译、模型导出、运行调试 |

---

## 🎯 学习路线建议

### 第一阶段：ROS2 与 C++ 基础（1~3 天）
1. 先读 **01 ROS2 基础**，理解 ROS2 的通信范式
2. 再读 **02 ROS2 C++ 节点开发**，跟着写一个简单的发布/订阅节点
3. 配合 **05 C++ 现代特性**，熟悉代码中大量使用的智能指针、Lambda、线程

### 第二阶段：视觉与推理核心（3~5 天）
4. **07 OpenCV 与计算机视觉** —— 理解图像处理和相机几何
5. **06 CUDA 与 TensorRT** —— 理解 GPU 预处理与神经网络推理
6. **11 Eigen 与数学计算** + **12 Open3D** —— 理解位姿解算的数学基础

### 第三阶段：算法与架构（3~5 天）
7. **08 目标跟踪与卡尔曼滤波** —— 这是项目的"大脑"
8. **14 项目架构与流水线** —— 把前面的知识点串成完整系统
9. **04 ROS2 Launch 与组件化** —— 理解如何启动和优化整个系统

### 第四阶段：工程化与部署（1~2 天）
10. **09 YAML 配置管理** + **10 Qt5 可视化开发** —— 交互与配置
11. **13 CMake 构建系统** —— 能独立修改编译规则
12. **15 从零部署项目** —— 完整走一遍部署流程

---

## 🔑 核心知识点速查

| 技术领域 | 本项目中的典型应用 |
|:---|:---|
| **ROS2** | VideoNode → DetectNode → PoseNode → MapNode 的流水线通信 |
| **CUDA** | `preprocess.cu` 中 GPU 并行图像预处理（resize + normalize） |
| **TensorRT** | `model.cpp` 中 `.engine` 模型的反序列化与异步推理 |
| **OpenCV** | 图像捕获、PnP 标定、`cv_bridge` 与 ROS 图像消息互转 |
| **Eigen** | `kalman.hpp` 中 8×8 和 4×4 固定矩阵的 Kalman 迭代 |
| **匈牙利算法** | `hungarian.cpp` 中检测框与固定槽位的最优匹配 |
| **Qt5** | `qt_display_node.cpp` 中多窗口可视化与交互按钮 |
| **yaml-cpp** | `ConfigManager.cpp` 中读取 camera.yaml / model.yaml |

---

## 📂 本项目文件对应关系

```
src/tensorrt_detect/
├── apps/standalone_main.cpp          → 独立运行入口（不依赖 ROS2）
├── src/nodes/
│   ├── video_node.cpp                → ROS2 图像源节点
│   ├── detect_node.cpp               → 检测节点（TensorRT 推理）
│   ├── pose_node.cpp                 → 位姿解算节点（PnP + Tracker）
│   ├── map_node.cpp                  → 小地图节点（OpenCV 绘图）
│   ├── qt_display_node.cpp           → Qt5 可视化节点
│   ├── calibrate_node.cpp            → 交互式相机标定节点
│   └── roi_set_node.cpp              → 交互式 ROI 框定节点
├── src/core/
│   ├── model.cpp / model.hpp         → TensorRT 推理封装
│   ├── pipeline.cpp / pipeline.hpp   → 多模型级联检测流水线
│   ├── preprocess.cu                 → CUDA 图像预处理
│   ├── kalman.cpp / kalman.hpp       → 卡尔曼滤波器
│   ├── tracker.cpp / tracker.hpp     → 多目标跟踪器
│   ├── hungarian.cpp / hungarian.hpp → 匈牙利匹配算法
│   ├── posesolver.cpp / posesolver.hpp → PnP + Raycasting 位姿解算
│   ├── raycaster.cpp / raycaster.hpp → Open3D 射线投影
│   ├── bot_identity.cpp / bot_identity.hpp → 身份稳定池
│   └── ...
├── src/visualization/                → OpenCV / Qt 绘图工具
├── src/config/ConfigManager.cpp      → YAML 配置管理器
├── launch/detect_pipeline.launch.py  → 系统启动文件
└── CMakeLists.txt                    → 构建配置
```

---

> 💡 **提示**：本文档系列的所有代码示例均直接取自本项目源码，确保学以致用。建议边读边在源码中搜索对应关键字，加深理解。
