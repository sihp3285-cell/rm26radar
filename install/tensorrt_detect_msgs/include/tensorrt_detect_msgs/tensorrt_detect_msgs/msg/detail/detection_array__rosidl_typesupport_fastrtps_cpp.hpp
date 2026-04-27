// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from tensorrt_detect_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tensorrt_detect_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "tensorrt_detect_msgs/msg/detail/detection_array__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace tensorrt_detect_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
cdr_serialize(
  const tensorrt_detect_msgs::msg::DetectionArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  tensorrt_detect_msgs::msg::DetectionArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
get_serialized_size(
  const tensorrt_detect_msgs::msg::DetectionArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
max_serialized_size_DetectionArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
cdr_serialize_key(
  const tensorrt_detect_msgs::msg::DetectionArray & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
get_serialized_size_key(
  const tensorrt_detect_msgs::msg::DetectionArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
max_serialized_size_key_DetectionArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace tensorrt_detect_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tensorrt_detect_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tensorrt_detect_msgs, msg, DetectionArray)();

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
