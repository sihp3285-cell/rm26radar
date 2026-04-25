// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/radar_map.h"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__STRUCT_H_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__STRUCT_H_

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

/// Struct defined in msg/RadarMap in the package tensorrt_detect_msgs.
typedef struct tensorrt_detect_msgs__msg__RadarMap
{
  std_msgs__msg__Header header;
  /// 蓝方 1-5 号及哨兵(6)的地图坐标 (x, y)
  float blue_x[6];
  float blue_y[6];
  /// 红方 1-5 号及哨兵(6)的地图坐标 (x, y)
  float red_x[6];
  float red_y[6];
} tensorrt_detect_msgs__msg__RadarMap;

// Struct for a sequence of tensorrt_detect_msgs__msg__RadarMap.
typedef struct tensorrt_detect_msgs__msg__RadarMap__Sequence
{
  tensorrt_detect_msgs__msg__RadarMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tensorrt_detect_msgs__msg__RadarMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__STRUCT_H_
