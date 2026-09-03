cmake_minimum_required(VERSION 3.8)
project(rm_field)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Header-only 契约包：不编译任何源文件。
add_library(rm_field INTERFACE)
target_include_directories(rm_field INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS rm_field EXPORT export_rm_field)
ament_export_targets(export_rm_field HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
