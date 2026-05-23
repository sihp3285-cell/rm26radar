# 04 ROS2 Launch 与组件化

> Launch 文件是 ROS2 系统的"启动脚本"，负责一次性启动多个节点、加载参数、配置话题重映射。而 **Component（组件）** 机制允许将多个节点编译到同一进程中，结合**进程内零拷贝**大幅提升图像传输效率。本章以本项目的 `detect_pipeline.launch.py` 为核心，深入讲解这些高级特性。

---

## 4.1 Launch 文件基础

### 4.1.1 Python Launch 文件结构

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tensorrt_detect',
            executable='video_node',
            name='video_node',
            output='screen',
        ),
    ])
```

### 4.1.2 参数文件路径解析

```python
params_file = PathJoinSubstitution([
    FindPackageShare('tensorrt_detect'),  # 查找包的 share 目录
    'config',
    'ros2_params.yaml',
])
```

`FindPackageShare` 会自动解析 `install/tensorrt_detect/share/tensorrt_detect/` 路径，确保在不同工作空间下都能正确找到参数文件。

---

## 4.2 Node vs ComposableNode

### 4.2.1 普通 Node（独立进程）

```python
Node(
    package='tensorrt_detect',
    executable='calibrate_node',   # 独立可执行文件
    name='calibrate_node',
    output='screen',
    parameters=[params_file],
)
```

每个 `Node` 启动一个**独立进程**，节点间通过 DDS 传输数据，涉及序列化和网络栈。

### 4.2.2 ComposableNode（组件，共享进程）

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

pipeline_container = ComposableNodeContainer(
    name='detect_pipeline_container',
    namespace='',
    package='rclcpp_components',
    executable='component_container',  # 进程入口
    composable_node_descriptions=[
        ComposableNode(
            package='tensorrt_detect',
            plugin='VideoNode',          # 类名（必须注册过）
            name='video_node',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='tensorrt_detect',
            plugin='DetectNode',
            name='detect_node',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='tensorrt_detect',
            plugin='PoseNode',
            name='pose_node',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
        ComposableNode(
            package='tensorrt_detect',
            plugin='MapNode',
            name='map_node',
            parameters=[params_file],
            extra_arguments=[{'use_intra_process_comms': True}],
        ),
    ],
    output='screen',
)
```

| 特性 | Node | ComposableNode |
|:---|:---|:---|
| 进程数 | 每个节点一个 | 多个节点共享一个 |
| 内存占用 | 较高 | 较低 |
| 数据传输 | DDS 序列化 | **进程内共享指针**（零拷贝） |
| 容错性 | 单个节点崩溃不影响其他 | 单个节点崩溃可能导致整个容器崩溃 |
| 适用场景 | I/O 阻塞型、UI 节点 | 高频数据流水线 |

---

## 4.3 Component 的 C++ 注册

要在 Launch 中使用 `plugin='VideoNode'`，必须在 C++ 源码中注册：

```cpp
// src/nodes/video_node.cpp
#include <rclcpp_components/register_node_macro.hpp>

class VideoNode : public rclcpp::Node {
    // ...
};

// 注册为可加载组件
RCLCPP_COMPONENTS_REGISTER_NODE(VideoNode)
```

### 4.3.1 CMake 配置

```cmake
# 编译为 SHARED 库（不是可执行文件）
add_library(video_node_component SHARED src/nodes/video_node.cpp)
ament_target_dependencies(video_node_component 
    rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs)
target_link_libraries(video_node_component pthread ${OpenCV_LIBS})
target_include_directories(video_node_component PRIVATE ${COMMON_INCLUDE_DIRS})

# 注册组件
rclcpp_components_register_nodes(video_node_component "VideoNode")

# 安装到 lib/ 根目录（ament_index 才能找到）
install(TARGETS video_node_component DESTINATION lib)
```

---

## 4.4 本项目的 Launch 设计解析

### 4.4.1 完整 `detect_pipeline.launch.py`

```python
def generate_launch_description():
    # 参数文件路径
    params_file = PathJoinSubstitution([
        FindPackageShare('tensorrt_detect'), 'config', 'ros2_params.yaml',
    ])

    # === 核心流水线：进程内零拷贝容器 ===
    # 使用单线程容器，避免 TensorRT 与 Open3D RaycastingScene 并发 CUDA 操作导致 SIGSEGV
    pipeline_container = ComposableNodeContainer(
        name='detect_pipeline_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(package='tensorrt_detect', plugin='VideoNode',
                           name='video_node', parameters=[params_file],
                           extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(package='tensorrt_detect', plugin='DetectNode',
                           name='detect_node', parameters=[params_file],
                           extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(package='tensorrt_detect', plugin='PoseNode',
                           name='pose_node', parameters=[params_file],
                           extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(package='tensorrt_detect', plugin='MapNode',
                           name='map_node', parameters=[params_file],
                           extra_arguments=[{'use_intra_process_comms': True}]),
        ],
        output='screen',
    )

    return LaunchDescription([
        pipeline_container,

        # 标定节点：独立进程，含交互式 OpenCV 窗口
        Node(package='tensorrt_detect', executable='calibrate_node',
             name='calibrate_node', output='screen', parameters=[params_file]),

        # Qt 显示节点：独立进程，含 Qt 事件循环
        Node(package='tensorrt_detect', executable='qt_display_node',
             name='qt_display_node', output='screen', parameters=[params_file]),

        # ROI 设置节点：独立进程，含交互式 OpenCV 窗口
        Node(package='tensorrt_detect', executable='roi_set_node',
             name='roi_set_node', output='screen', parameters=[params_file]),
    ])
```

### 4.4.2 设计决策分析

| 节点 | 进程模式 | 理由 |
|:---|:---|:---|
| VideoNode / DetectNode / PoseNode / MapNode | **ComposableNodeContainer** | 高频图像/检测数据流，必须零拷贝降低延迟 |
| CalibrateNode | **独立 Node** | 含 OpenCV 交互窗口（`cv::imshow` + `cv::waitKey`），需要独立 GUI 线程 |
| QtDisplayNode | **独立 Node** | 含 Qt 事件循环（`QApplication::exec`），与 ROS spin 需分离线程 |
| ROISetNode | **独立 Node** | 同 CalibrateNode，含交互式 OpenCV 窗口 |

### 4.4.3 为什么使用 `component_container` 而非 `component_container_mt`

```python
# 单线程容器（本项目使用）
executable='component_container'

# 多线程容器（未使用）
# executable='component_container_mt'
```

**原因**：`DetectNode`（TensorRT CUDA 推理）和 `PoseNode`（Open3D Raycasting CUDA 操作）如果并发执行 CUDA kernel，会导致 **SIGSEGV**。使用单线程容器**序列化回调执行**，从根本上避免 CUDA 并发冲突。

> 本项目还通过全局 CUDA 互斥锁进一步保护：
> ```cpp
> namespace cuda_guard {
>     inline std::mutex& getCudaMutex() {
>         static std::mutex mtx;
>         return mtx;
>     }
> }
> ```

---

## 4.5 Executor 详解

### 4.5.1 单线程 Executor

```cpp
rclcpp::spin(node);  // 等价于：
// rclcpp::executors::SingleThreadedExecutor executor;
// executor.add_node(node);
// executor.spin();
```

- 所有回调在同一个线程顺序执行
- 无并发问题，但一个耗时回调会阻塞其他回调

### 4.5.2 多线程 Executor

```cpp
rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(), 2);  // 2 线程
executor.add_node(node);
executor.spin();
```

- 回调在多个线程中并发执行
- 适合 I/O 等待型场景
- **需要自行保证线程安全**（加锁）

### 4.5.3 静态单线程 Executor（ROS2 Humble+）

```cpp
rclcpp::executors::StaticSingleThreadedExecutor executor;
executor.add_node(node);
executor.spin();
```

- 编译时确定回调拓扑，运行时开销更小
- 适用于拓扑固定的实时系统

---

## 4.6 动态配置重载

本项目通过 **Service** 实现运行时不重启节点的配置更新：

### 4.6.1 标定重载流程

```
用户点击 Qt 界面 "重新标定" 按钮
    ↓
QtDisplayNode 调用 /calibration/start（异步 service 调用）
    ↓
CalibrateNode 打开 OpenCV 窗口，用户点击标定点
    ↓
CalibrateNode 计算 R/T，保存到 calib_result.yaml
    ↓
CalibrateNode 调用 /pose_node/reload_calibration
    ↓
PoseNode 重新加载 R/T，继续处理新帧
```

### 4.6.2 ROI 重载流程

```
用户点击 Qt 界面 "重新框定 ROI"
    ↓
QtDisplayNode 调用 /roi_set/start
    ↓
ROISetNode 打开 OpenCV 窗口，用户框选 ROI
    ↓
ROISetNode 保存到 outpost_roi.yaml
    ↓
ROISetNode 调用 /detect_node/reload_roi
    ↓
DetectNode 重新读取 outpost_roi.yaml
```

---

## 4.7 本章小结

- Launch 文件使用 Python 编写，通过 `Node` 和 `ComposableNode` 启动节点
- `ComposableNodeContainer` 将多个节点放入同一进程，启用 `use_intra_process_comms` 实现零拷贝
- 含 GUI（OpenCV/Qt）的节点必须作为独立 `Node` 运行，避免事件循环冲突
- 单线程容器 `component_container` 可序列化回调，解决 CUDA 并发 SIGSEGV
- Service + 临时节点的模式实现了运行时的动态配置重载
