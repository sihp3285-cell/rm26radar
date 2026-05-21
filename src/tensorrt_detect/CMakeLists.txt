cmake_minimum_required(VERSION 3.22.1)
project(tensorrt_detect LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Release")

if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 86)
endif()

# 确保安装后的可执行文件能自动找到 Open3D 等第三方动态库
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0/lib")

# ========== ROS2 依赖 ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tensorrt_detect_msgs REQUIRED)

# --- 修正 Open3D 查找方式 ---
list(APPEND CMAKE_PREFIX_PATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0")
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Open3D REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# --- TensorRT 部分保持不变 ---
set(TRT_ROOT "/opt/TensorRT/TensorRT-10.2.0.19")
add_library(tensorrt_interface INTERFACE)
target_include_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/include)
target_link_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/lib)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-deprecated-declarations)
    add_compile_options(-O3 -march=native)
endif()

# ========== 公共头文件搜索路径 ==========
set(COMMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt_detect
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt_detect/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/tensorrt_detect/visualization
)

# ========== 内部核心库：tensorrt_detect_core ==========
# 聚合所有公共算法、可视化、配置源码，后续节点只需链接此库即可
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
    src/core/preprocess.cu
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
# 静态库被链接进共享库（component），需要 -fPIC
set_target_properties(tensorrt_detect_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ========== 原程序（独立运行） ==========
add_executable(standard apps/standalone_main.cpp)

target_link_libraries(standard
    PRIVATE
    tensorrt_detect_core
)

# ========== Component libraries for intra-process zero-copy ==========

# video_node component
add_library(video_node_component SHARED src/nodes/video_node.cpp)
ament_target_dependencies(video_node_component rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs)
target_link_libraries(video_node_component pthread ${OpenCV_LIBS})
target_include_directories(video_node_component PRIVATE ${COMMON_INCLUDE_DIRS})
rclcpp_components_register_nodes(video_node_component "VideoNode")

# detect_node component
add_library(detect_node_component SHARED src/nodes/detect_node.cpp)
ament_target_dependencies(detect_node_component rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(detect_node_component tensorrt_detect_core)
rclcpp_components_register_nodes(detect_node_component "DetectNode")

# pose_node component
add_library(pose_node_component SHARED src/nodes/pose_node.cpp)
ament_target_dependencies(pose_node_component rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(pose_node_component tensorrt_detect_core)
rclcpp_components_register_nodes(pose_node_component "PoseNode")

# map_node component
add_library(map_node_component SHARED src/nodes/map_node.cpp)
ament_target_dependencies(map_node_component rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs tensorrt_detect_msgs)
target_link_libraries(map_node_component tensorrt_detect_core)
rclcpp_components_register_nodes(map_node_component "MapNode")

# Standalone executables (backward compatibility)
add_executable(video_node src/nodes/video_node.cpp)
ament_target_dependencies(video_node rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs)
target_link_libraries(video_node pthread ${OpenCV_LIBS})
target_include_directories(video_node PRIVATE ${COMMON_INCLUDE_DIRS})

add_executable(detect_node src/nodes/detect_node.cpp)
ament_target_dependencies(detect_node rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(detect_node tensorrt_detect_core)

add_executable(pose_node src/nodes/pose_node.cpp)
ament_target_dependencies(pose_node rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(pose_node tensorrt_detect_core)

add_executable(map_node src/nodes/map_node.cpp)
ament_target_dependencies(map_node rclcpp rclcpp_components sensor_msgs cv_bridge std_msgs tensorrt_detect_msgs)
target_link_libraries(map_node tensorrt_detect_core)

# ========== ROS2 节点：display_node ==========
add_executable(display_node src/nodes/display_node.cpp)
ament_target_dependencies(display_node rclcpp sensor_msgs cv_bridge std_msgs)
target_link_libraries(display_node pthread ${OpenCV_LIBS})
target_include_directories(display_node PRIVATE ${COMMON_INCLUDE_DIRS})

# ========== ROS2 节点：calibrate_node ==========
add_executable(calibrate_node src/nodes/calibrate_node.cpp)
ament_target_dependencies(calibrate_node rclcpp sensor_msgs cv_bridge std_msgs std_srvs)
target_link_libraries(calibrate_node tensorrt_detect_core)
target_include_directories(calibrate_node PRIVATE ${COMMON_INCLUDE_DIRS})

# ========== ROS2 节点：roi_set_node ==========
add_executable(roi_set_node src/nodes/roi_set_node.cpp)
ament_target_dependencies(roi_set_node rclcpp sensor_msgs cv_bridge std_msgs std_srvs)
target_link_libraries(roi_set_node tensorrt_detect_core)
target_include_directories(roi_set_node PRIVATE ${COMMON_INCLUDE_DIRS})

# ========== ROS2 节点：qt_display_node ==========
add_executable(qt_display_node src/nodes/qt_display_node.cpp)
ament_target_dependencies(qt_display_node rclcpp sensor_msgs cv_bridge std_msgs std_srvs tensorrt_detect_msgs)
target_link_libraries(qt_display_node tensorrt_detect_core Qt5::Core Qt5::Widgets)

# ========== 安装 & ament 打包 ==========
# 可执行文件安装到 lib/${PROJECT_NAME}/
install(TARGETS
    standard
    detect_node
    display_node
    video_node
    pose_node
    map_node
    calibrate_node
    roi_set_node
    qt_display_node
    DESTINATION lib/${PROJECT_NAME}
)

# component library 必须安装到 lib/ 根目录，否则 ament_index 路径对不上
install(TARGETS
    video_node_component
    detect_node_component
    pose_node_component
    map_node_component
    DESTINATION lib
)

# ========== 安装 launch 文件 ==========
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# ========== 安装参数配置文件 ==========
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
