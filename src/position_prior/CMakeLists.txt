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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_coordinate_transform test/test_coordinate_transform.cpp)
  target_link_libraries(test_coordinate_transform position_prior_core)

  ament_add_gtest(test_model_golden test/test_model_golden.cpp)
  target_link_libraries(test_model_golden position_prior_core yaml-cpp::yaml-cpp)
  target_compile_definitions(test_model_golden PRIVATE
    POSITION_PRIOR_TOOLKIT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../position_prior_toolkit")

  ament_add_gtest(test_prior_gate test/test_prior_gate.cpp)
  target_link_libraries(test_prior_gate position_prior_core)

  ament_add_gtest(test_prior_lifecycle test/test_prior_lifecycle.cpp)
  target_link_libraries(test_prior_lifecycle position_prior_core)

  ament_add_gtest(test_observation_confirmation test/test_observation_confirmation.cpp)
  target_link_libraries(test_observation_confirmation position_prior_core)

  ament_add_gtest(test_team_selection test/test_team_selection.cpp)
  target_link_libraries(test_team_selection position_prior_core)
endif()

ament_package()
