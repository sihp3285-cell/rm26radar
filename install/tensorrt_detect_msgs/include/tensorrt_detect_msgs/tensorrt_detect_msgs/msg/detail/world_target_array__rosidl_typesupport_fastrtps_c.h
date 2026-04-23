// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice
#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tensorrt_detect_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "tensorrt_detect_msgs/msg/detail/world_target_array__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
bool cdr_serialize_tensorrt_detect_msgs__msg__WorldTargetArray(
  const tensorrt_detect_msgs__msg__WorldTargetArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
bool cdr_deserialize_tensorrt_detect_msgs__msg__WorldTargetArray(
  eprosima::fastcdr::Cdr &,
  tensorrt_detect_msgs__msg__WorldTargetArray * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
size_t get_serialized_size_tensorrt_detect_msgs__msg__WorldTargetArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
size_t max_serialized_size_tensorrt_detect_msgs__msg__WorldTargetArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
bool cdr_serialize_key_tensorrt_detect_msgs__msg__WorldTargetArray(
  const tensorrt_detect_msgs__msg__WorldTargetArray * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
size_t get_serialized_size_key_tensorrt_detect_msgs__msg__WorldTargetArray(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
size_t max_serialized_size_key_tensorrt_detect_msgs__msg__WorldTargetArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_tensorrt_detect_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, tensorrt_detect_msgs, msg, WorldTargetArray)();

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
