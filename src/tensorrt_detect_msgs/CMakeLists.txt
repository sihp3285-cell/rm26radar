cmake_minimum_required(VERSION 3.22.1)
project(tensorrt_detect_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/DetectionBox.msg
  msg/DetectionArray.msg
  msg/WorldTarget.msg
  msg/WorldTargetArray.msg
  msg/RadarMap.msg
  msg/MapTactics.msg
  msg/PipelineTiming.msg
  msg/PriorCandidate.msg
  msg/PriorPrediction.msg
  msg/PriorPredictionArray.msg
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
