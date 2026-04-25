cmake_minimum_required(VERSION 3.22.1)
project(tensorrt_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

# 确保安装后的可执行文件能自动找到 Open3D 等第三方动态库
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(APPEND CMAKE_INSTALL_RPATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0/lib")

# ========== ROS2 依赖 ==========
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tensorrt_detect_msgs REQUIRED)

# --- 修正 Open3D 查找方式 ---
list(APPEND CMAKE_PREFIX_PATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0")
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Open3D REQUIRED)

# --- TensorRT 部分保持不变 ---
set(TRT_ROOT "/opt/TensorRT/TensorRT-10.2.0.19")
add_library(tensorrt_interface INTERFACE)
target_include_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/include)
target_link_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/lib)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-deprecated-declarations)
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
    src/visualization/draw.cpp
    src/visualization/ui.cpp
    src/config/ConfigManager.cpp
)

target_include_directories(tensorrt_detect_core PUBLIC ${COMMON_INCLUDE_DIRS})

target_link_libraries(tensorrt_detect_core
    PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    curl
    tensorrt_interface
    Open3D::Open3D
    PRIVATE
    pthread
    nvinfer
    nvinfer_plugin
    cudart
)

# ========== 原程序（独立运行） ==========
add_executable(standard apps/standalone_main.cpp)

target_link_libraries(standard
    PRIVATE
    tensorrt_detect_core
)

# ========== ROS2 节点：detect_node ==========
add_executable(detect_node src/nodes/detect_node.cpp)

ament_target_dependencies(detect_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    tensorrt_detect_msgs
)

target_link_libraries(detect_node tensorrt_detect_core)

# ========== ROS2 节点：video_node ==========
add_executable(video_node src/nodes/video_node.cpp)

ament_target_dependencies(video_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
)

target_link_libraries(video_node pthread ${OpenCV_LIBS})

target_include_directories(video_node PRIVATE ${COMMON_INCLUDE_DIRS})

# ========== ROS2 节点：display_node ==========
add_executable(display_node src/nodes/display_node.cpp)

ament_target_dependencies(display_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
)

target_link_libraries(display_node pthread ${OpenCV_LIBS})

target_include_directories(display_node PRIVATE ${COMMON_INCLUDE_DIRS})

# ========== ROS2 节点：pose_node ==========
add_executable(pose_node src/nodes/pose_node.cpp)

ament_target_dependencies(pose_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    tensorrt_detect_msgs
)

target_link_libraries(pose_node tensorrt_detect_core)

# ========== ROS2 节点：map_node ==========
add_executable(map_node src/nodes/map_node.cpp)

ament_target_dependencies(map_node
    rclcpp
    sensor_msgs
    cv_bridge
    std_msgs
    tensorrt_detect_msgs
)

target_link_libraries(map_node tensorrt_detect_core)

# ========== 安装 & ament 打包 ==========
install(TARGETS
    standard
    detect_node
    display_node
    video_node
    pose_node
    map_node
    DESTINATION lib/${PROJECT_NAME}
)

# ========== 安装 launch 文件 ==========
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
