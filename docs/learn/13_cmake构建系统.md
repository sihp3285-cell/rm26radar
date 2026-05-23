# 13 CMake 构建系统

> 本项目使用 `ament_cmake` 作为 ROS2 的构建工具，同时集成 CUDA、TensorRT、Open3D、Qt5 等多个第三方库。CMakeLists.txt 的设计直接影响编译速度、可维护性和跨平台兼容性。本章逐段解析本项目的构建配置。

---

## 13.1 ament_cmake 基础

### 13.1.1 与普通 CMake 的区别

| 特性 | 纯 CMake | ament_cmake（ROS2） |
|:---|:---|:---|
| 包发现 | `find_package()` | `find_package()` + `ament_target_dependencies()` |
| 安装规则 | 手动 `install()` | `ament_package()` 自动处理部分路径 |
| 索引注册 | 无 | 自动生成 `ament_index` |
| 依赖导出 | 手动 | `ament_export_dependencies()` |

### 13.1.2 最小 ament_cmake 包

```cmake
cmake_minimum_required(VERSION 3.22.1)
project(my_package)

find_package(ament_cmake REQUIRED)

add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

install(TARGETS my_node DESTINATION lib/${PROJECT_NAME})
ament_package()
```

---

## 13.2 本项目 CMakeLists.txt 逐段解析

### 13.2.1 项目声明与标准设置

```cmake
cmake_minimum_required(VERSION 3.22.1)
project(tensorrt_detect LANGUAGES CXX CUDA)  # ← 声明 CUDA 语言

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成 compile_commands.json，供 LSP 使用
set(CMAKE_BUILD_TYPE "Release")

# CUDA 架构：SM 86（Ampere）
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 86)
endif()
```

### 13.2.2 RPATH 配置

```cmake
# 确保安装后的可执行文件能找到 Open3D 等第三方动态库
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0/lib")
```

**RPATH** 是 ELF 可执行文件中的动态库搜索路径。设置后，运行时无需 `LD_LIBRARY_PATH` 也能找到库。

### 13.2.3 ROS2 依赖查找

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tensorrt_detect_msgs REQUIRED)
```

### 13.2.4 第三方库查找

```cmake
# Open3D 前缀路径
list(APPEND CMAKE_PREFIX_PATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0")
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Open3D REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
```

### 13.2.5 TensorRT 接口库

```cmake
set(TRT_ROOT "/opt/TensorRT/TensorRT-10.2.0.19")
add_library(tensorrt_interface INTERFACE)  # 头文件-only 接口库
target_include_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/include)
target_link_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/lib)
```

**INTERFACE 库**：不编译任何源文件，仅传递包含路径和链接路径给使用者。

### 13.2.6 编译优化选项

```cmake
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-deprecated-declarations)  # 忽略弃用警告
    add_compile_options(-O3 -march=native)              # 最高优化 + 针对本机 CPU 指令集
endif()
```

| 选项 | 作用 |
|:---|:---|
| `-O3` | 最高级别优化（循环展开、函数内联、向量化） |
| `-march=native` | 针对当前 CPU 生成最优指令（AVX2 等） |
| `-Wno-deprecated-declarations` | 忽略 deprecated API 警告（TensorRT 某些旧接口） |

### 13.2.7 公共头文件路径

```cmake
set(COMMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt_detect
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt_detect/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt_detect/visualization
)
```

### 13.2.8 核心静态库

```cmake
add_library(tensorrt_detect_core STATIC
    src/core/model.cpp
    src/core/pipeline.cpp
    src/core/posesolver.cpp
    src/core/mouseback.cpp
    src/core/radarmap.cpp
    src/core/raycaster.cpp
    src/core/map_analyzer.cpp
    src/core/kalman.cpp
    src/core/tracker.cpp
    src/core/bot_identity.cpp
    src/core/hungarian.cpp
    src/core/preprocess.cu        # ← CUDA 源文件
    src/visualization/draw.cpp
    src/visualization/ui.cpp
    src/config/ConfigManager.cpp
)

target_include_directories(tensorrt_detect_core PUBLIC ${COMMON_INCLUDE_DIRS})

target_link_libraries(tensorrt_detect_core
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    curl
    tensorrt_interface
    Open3D::Open3D
    tensorrt_detect_msgs::tensorrt_detect_msgs__rosidl_generator_cpp
    pthread
    nvinfer
    nvinfer_plugin
    cudart
)

# 静态库被链接进共享库（component），需要 -fPIC（位置无关代码）
set_target_properties(tensorrt_detect_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
```

**`-fPIC` 的必要性**：
- `tensorrt_detect_core` 是 STATIC 库
- 它被链接到 `detect_node_component` 等 SHARED 库中
- Linux 下共享库中的所有代码必须是位置无关的（PIC）

### 13.2.9 独立可执行文件

```cmake
add_executable(standard apps/standalone_main.cpp)
target_link_libraries(standard PRIVATE tensorrt_detect_core)
```

`standard` 是不依赖 ROS2 的独立运行程序，用于快速测试推理流水线。

### 13.2.10 Component 共享库

```cmake
add_library(detect_node_component SHARED src/nodes/detect_node.cpp)
ament_target_dependencies(detect_node_component 
    rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(detect_node_component tensorrt_detect_core)
rclcpp_components_register_nodes(detect_node_component "DetectNode")

install(TARGETS detect_node_component DESTINATION lib)
```

**关键区别**：
- Component 是 `SHARED` 库，不是 `EXECUTABLE`
- `rclcpp_components_register_nodes` 生成注册表，让 Launch 的 `plugin='DetectNode'` 能工作
- 安装到 `lib/` 根目录（不是 `lib/tensorrt_detect/`），否则 `ament_index` 找不到

### 13.2.11 普通节点可执行文件（向后兼容）

```cpp
add_executable(detect_node src/nodes/detect_node.cpp)
ament_target_dependencies(detect_node ...)
target_link_libraries(detect_node tensorrt_detect_core)
```

这些可执行文件用于独立运行节点（`ros2 run tensorrt_detect detect_node`）。

### 13.2.12 Qt5 节点

```cmake
add_executable(qt_display_node src/nodes/qt_display_node.cpp)
ament_target_dependencies(qt_display_node rclcpp sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(qt_display_node tensorrt_detect_core Qt5::Core Qt5::Widgets)
```

Qt 节点需要显式链接 `Qt5::Core` 和 `Qt5::Widgets`。

### 13.2.13 安装规则

```cmake
# 可执行文件安装到 lib/${PROJECT_NAME}/
install(TARGETS standard detect_node display_node video_node pose_node map_node
                calibrate_node roi_set_node qt_display_node
        DESTINATION lib/${PROJECT_NAME})

# component 库安装到 lib/ 根目录
install(TARGETS video_node_component detect_node_component pose_node_component map_node_component
        DESTINATION lib)

# 安装 launch 文件
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# 安装参数配置文件
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
```

---

## 13.3 自定义消息包的 CMake

```cmake
cmake_minimum_required(VERSION 3.22.1)
project(tensorrt_detect_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DetectionBox.msg
  msg/DetectionArray.msg
  msg/WorldTarget.msg
  msg/WorldTargetArray.msg
  msg/RadarMap.msg
  msg/MapTactics.msg
  msg/PipelineTiming.msg
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

---

## 13.4 常见编译问题

### 13.4.1 CUDA 架构不匹配

```
error: no kernel image is available for execution on the device
```

**解决**：检查 `CMAKE_CUDA_ARCHITECTURES` 与显卡 SM 版本是否匹配：

```bash
nvidia-smi  # 查看显卡型号
# RTX 3060/3070/3080 → SM 86
# RTX 4090 → SM 89
```

### 13.4.2 TensorRT 路径错误

```
error: NvInfer.h: No such file or directory
```

**解决**：修改 `TRT_ROOT` 为实际安装路径。

### 13.4.3 Open3D 链接失败

```
error: cannot find -lOpen3D
```

**解决**：检查 `CMAKE_PREFIX_PATH` 和 `CMAKE_INSTALL_RPATH`。

### 13.4.4 yaml-cpp 版本冲突

```
undefined reference to `YAML::LoadFile(std::string const&)`
```

**解决**：yaml-cpp 0.8.0 使用 `yaml-cpp::yaml-cpp` target，旧版本可能需要链接 `yaml-cpp`。

---

## 13.5 本章小结

| 技术点 | 作用 | 关键配置 |
|:---|:---|:---|
| `LANGUAGES CXX CUDA` | 启用 CUDA 编译 | `project(... LANGUAGES CXX CUDA)` |
| `CMAKE_CUDA_ARCHITECTURES` | 指定 GPU 架构 | `set(CMAKE_CUDA_ARCHITECTURES 86)` |
| INTERFACE 库 | 传递头文件/库路径 | `add_library(tensorrt_interface INTERFACE)` |
| `-fPIC` | 静态库链接到共享库 | `POSITION_INDEPENDENT_CODE ON` |
| Component 注册 | Launch 加载插件 | `rclcpp_components_register_nodes` |
| RPATH | 运行时库搜索路径 | `CMAKE_INSTALL_RPATH_USE_LINK_PATH` |
| `ament_target_dependencies` | 自动处理 ROS2 依赖 | 替代部分 `target_link_libraries` |
