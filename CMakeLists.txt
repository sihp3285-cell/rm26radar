cmake_minimum_required(VERSION 3.22.1)
project(tensorrt_detect)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE "Debug")

# --- 修正 Open3D 查找方式 ---
# 方法一：将 Open3D 安装根目录添加到 CMAKE_PREFIX_PATH（推荐）
list(APPEND CMAKE_PREFIX_PATH "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0")

# 方法二：仍用 Open3D_DIR，但需要确保它指向包含 .cmake 文件的子目录
# set(Open3D_DIR "/home/delphine/open3d-devel-linux-x86_64-cxx11-abi-cuda-0.19.0/lib/cmake/Open3D")

# 其他依赖的路径
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4")

# 查找包（必须在设置路径之后）
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(Open3D REQUIRED)   # 现在应该能找到了

# --- TensorRT 部分保持不变 ---
set(TRT_ROOT "/opt/TensorRT/TensorRT-10.2.0.19")
add_library(tensorrt_interface INTERFACE)
target_include_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/include)
target_link_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/lib)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()

# 可执行文件
add_executable(standard
    main.cpp
    utils/model.cpp
    utils/draw.cpp
    utils/posesolver.cpp
    utils/mouseback.cpp
    utils/radarmap.cpp
    utils/pipeline.cpp
    utils/ui.cpp
    ConfigManager.cpp 
    utils/raycaster.cpp
)

# 链接库
target_link_libraries(standard
    PRIVATE
    pthread
    nvinfer
    nvinfer_plugin
    cudart
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
    curl
    tensorrt_interface
    Open3D::Open3D
)

target_include_directories(standard PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
)


