// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tensorrt_detect_msgs:msg/WorldTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target.h"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__STRUCT_H_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/WorldTarget in the package tensorrt_detect_msgs.
typedef struct tensorrt_detect_msgs__msg__WorldTarget
{
  int32_t idx;
  int32_t class_id;
  int32_t team_id;
  float score;
  bool valid;
  float world_x;
  float world_y;
  float world_z;
  int32_t bbox_x;
  int32_t bbox_y;
  int32_t bbox_w;
  int32_t bbox_h;
} tensorrt_detect_msgs__msg__WorldTarget;

// Struct for a sequence of tensorrt_detect_msgs__msg__WorldTarget.
typedef struct tensorrt_detect_msgs__msg__WorldTarget__Sequence
{
  tensorrt_detect_msgs__msg__WorldTarget * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tensorrt_detect_msgs__msg__WorldTarget__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__STRUCT_H_
