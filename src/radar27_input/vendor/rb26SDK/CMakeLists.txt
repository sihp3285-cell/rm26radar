cmake_minimum_required(VERSION 3.16)
project(rb26SDK)

include_directories(include)
include_directories(include/daheng)
include_directories(include/hik)

# ── 架构相关的 SDK 搜索路径 ──
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(MVS_LIB_DIR /opt/MVS/lib/64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(MVS_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/hik/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
)

# ── 条件检测：各相机 SDK 是否可用 ──

# Hikvision MVS SDK
find_library(MVCAMERA_LIB MvCameraControl HINTS ${MVS_LIB_DIR})
if(MVCAMERA_LIB)
    message(STATUS "rb26SDK: Found MvCameraControl → 启用 Hikvision 相机")
    set(HAS_HIK ON)
else()
    message(STATUS "rb26SDK: MvCameraControl 未找到，跳过 Hikvision 支持")
    set(HAS_HIK OFF)
endif()

# Daheng Galaxy SDK
find_library(GXIAPI_LIB gxiapi)
if(GXIAPI_LIB)
    message(STATUS "rb26SDK: Found gxiapi → 启用 Daheng 相机")
    set(HAS_DAHENG ON)
else()
    message(STATUS "rb26SDK: gxiapi 未找到，跳过 Daheng 支持")
    set(HAS_DAHENG OFF)
endif()

# ── 条件编译源文件 ──
set(RB26SDK_SOURCES)
if(HAS_HIK)
    list(APPEND RB26SDK_SOURCES ./src/hik/hik.cpp)
endif()
if(HAS_DAHENG)
    list(APPEND RB26SDK_SOURCES ./src/daheng/daheng.cpp)
endif()
if(NOT RB26SDK_SOURCES)
    message(FATAL_ERROR "rb26SDK: 没有可用的相机 SDK，至少需要 Hikvision 或 Daheng 之一")
endif()

add_library(rb26SDK ${RB26SDK_SOURCES})

target_include_directories(rb26SDK PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
                                           ${CMAKE_CURRENT_SOURCE_DIR}/include/daheng
                                           ${CMAKE_CURRENT_SOURCE_DIR}/include/hik
                                           )

# 静态库被链接进共享库（component），需要 -fPIC
set_target_properties(rb26SDK PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ── PUBLIC 编译定义：让消费者（camera_node 等）知道哪些后端可用 ──
if(HAS_HIK)
    target_compile_definitions(rb26SDK PUBLIC RB26SDK_HAS_HIK=1)
    target_link_directories(rb26SDK PUBLIC ${MVS_LIB_DIR})
endif()
if(HAS_DAHENG)
    target_compile_definitions(rb26SDK PUBLIC RB26SDK_HAS_DAHENG=1)
endif()

# ── 链接 ──
set(CAMERA_LIBS pthread ${OpenCV_LIBS})
if(HAS_HIK)
    list(APPEND CAMERA_LIBS ${MVCAMERA_LIB})
endif()
if(HAS_DAHENG)
    list(APPEND CAMERA_LIBS ${GXIAPI_LIB})
endif()

target_link_libraries(rb26SDK PUBLIC ${CAMERA_LIBS})
