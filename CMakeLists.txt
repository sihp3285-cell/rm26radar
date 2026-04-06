cmake_minimum_required(VERSION 3.22.1)#规定了CMake的最低版本要求，确保使用的CMake版本支持所需的功能和语法。
project(tensorrt_detect) # 定义了一个名为tensorrt_detect的项目，CMake将使用这个名称来组织和管理构建过程。

set(CMAKE_CXX_STANDARD 17) # 设置C++标准为C++17，确保代码可以使用C++17的特性和语法。
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 确保 CMake 会检查并要求使用指定的 C++ 标准。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # 导出编译命令，方便其他工具（如 clang-tidy）使用。
set(CMAKE_BUILD_TYPE "Debug") # 设置构建类型为 Debug，这会启用调试信息和断言检查。（决定编译模式，debug生成）
set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4") # 设置 OpenCV 库的查找路径，确保 CMake 可以找到 OpenCV 库。

# 1. 查找包
find_package(OpenCV REQUIRED) # 查找 OpenCV 库，确保它已安装并且 CMake 可以找到它。
find_package(yaml-cpp REQUIRED) # 查找 yaml-cpp 库，确保它已安装并且 CMake 可以找到它。
find_package(CUDAToolkit REQUIRED) # 推荐用 CUDAToolkit 替代旧的 CUDA 查找方式

# 2. TensorRT 路径变量 (方便修改)
set(TRT_ROOT "/opt/TensorRT/TensorRT-10.2.0.19") # 设置 TensorRT 库的根目录，确保 CMake 可以找到 TensorRT 库。
add_library(tensorrt_interface INTERFACE)
target_include_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/include)
target_link_directories(tensorrt_interface INTERFACE ${TRT_ROOT}/lib)

# 消除警告
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()

# 5. 定义可执行文件 (注意：只放 .cpp 文件，.hpp 文件不需要在这里列出)
add_executable(standard# 定义一个名为 standard 的可执行文件，包含以下源文件：
    main.cpp
    utils/model.cpp
    utils/draw.cpp
    utils/posesolver.cpp
    utils/mouseback.cpp
    utils/radarmap.cpp
    utils/pipeline.cpp
    utils/ui.cpp
    utils/tracker.cpp
)



# 6. 链接库
target_link_libraries(standard# 链接 standard 可执行文件需要的库：
    PRIVATE# 确保链接的库都是公开的，可在其他目标中使用
    pthread# 确保链接 pthread 库，用于多线程支持
    nvinfer# 确保链接 TensorRT 推理库
    nvinfer_plugin # 最好加上这个，TensorRT 插件库
    cudart# 确保链接 CUDA 运行时库
    ${OpenCV_LIBS}# 确保链接 OpenCV 库，用于图像处理和显示
    yaml-cpp::yaml-cpp# 确保链接 yaml-cpp 库，用于解析和生成 YAML 文件
    curl# 确保链接 curl 库，用于网络请求
    tensorrt_interface# 确保链接 tensorrt_interface 库，包含 TensorRT 的头文件和库路径
)
target_include_directories(standard PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}           # 当前目录 (找 ConfigManager.hpp)
    ${CMAKE_CURRENT_SOURCE_DIR}/utils     # utils目录
    ${CMAKE_CURRENT_SOURCE_DIR}/utils/include # 核心！(找 posesolver.hpp 和 mouseback.hpp)
)





