# Position Prior 包把无 ROS 依赖的模型/Gate/NavGrid 算法放在 core 库中，把 Topic、
# 参数、缓存生命周期和消息转换留给 position_prior_node。这样单元测试可直接验证
# 算法层，同时运行节点只需链接同一实现。
cmake_minimum_required(VERSION 3.22)
project(position_prior)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tensorrt_detect_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(position_prior_core
  src/blind_zone_prior.cpp
  src/navigation_mesh.cpp
  src/observation_confirmation.cpp
  src/coordinate_transform.cpp
  src/position_prior_model.cpp
  src/prior_gate.cpp
  src/prior_lifecycle.cpp
)
target_include_directories(position_prior_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(position_prior_core PUBLIC
  OpenSSL::Crypto
  yaml-cpp::yaml-cpp
)

add_executable(position_prior_node src/position_prior_node.cpp)
target_link_libraries(position_prior_node position_prior_core)
ament_target_dependencies(position_prior_node
  rclcpp
  std_msgs
  tensorrt_detect_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(TARGETS position_prior_core position_prior_node
  EXPORT export_position_prior
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_position_prior HAS_LIBRARY_TARGET)
ament_export_dependencies(OpenSSL yaml-cpp)

ament_package()
