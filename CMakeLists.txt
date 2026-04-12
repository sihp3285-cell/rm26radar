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

# 如果 Eigen3 或 fmt 的 CMake 查找失败，可用以下手动方式
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_path(EIGEN3_INCLUDE_DIR "Eigen/Core" PATHS /usr/include/eigen3)
    if(EIGEN3_INCLUDE_DIR)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIR})
    endif()
endif()

find_package(fmt QUIET)
if(NOT fmt_FOUND)
    find_library(FMT_LIBRARY fmt)
    find_path(FMT_INCLUDE_DIR "fmt/format.h" PATHS /usr/include)
    if(FMT_LIBRARY AND FMT_INCLUDE_DIR)
        add_library(fmt::fmt UNKNOWN IMPORTED)
        set_target_properties(fmt::fmt PROPERTIES
            IMPORTED_LOCATION ${FMT_LIBRARY}
            INTERFACE_INCLUDE_DIRECTORIES ${FMT_INCLUDE_DIR}
        )
    endif()
endif()

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
    utils/tracker.cpp
    ConfigManager.cpp 
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
    Eigen3::Eigen
    fmt::fmt
)

target_include_directories(standard PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include
)


