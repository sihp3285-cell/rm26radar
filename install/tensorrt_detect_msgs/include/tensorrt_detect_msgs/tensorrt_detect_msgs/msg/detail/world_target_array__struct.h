// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target_array.h"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__STRUCT_H_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'targets'
#include "tensorrt_detect_msgs/msg/detail/world_target__struct.h"

/// Struct defined in msg/WorldTargetArray in the package tensorrt_detect_msgs.
typedef struct tensorrt_detect_msgs__msg__WorldTargetArray
{
  std_msgs__msg__Header header;
  tensorrt_detect_msgs__msg__WorldTarget__Sequence targets;
} tensorrt_detect_msgs__msg__WorldTargetArray;

// Struct for a sequence of tensorrt_detect_msgs__msg__WorldTargetArray.
typedef struct tensorrt_detect_msgs__msg__WorldTargetArray__Sequence
{
  tensorrt_detect_msgs__msg__WorldTargetArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tensorrt_detect_msgs__msg__WorldTargetArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__STRUCT_H_
