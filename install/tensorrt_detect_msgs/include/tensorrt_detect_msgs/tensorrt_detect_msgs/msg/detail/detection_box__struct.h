// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tensorrt_detect_msgs:msg/DetectionBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/detection_box.h"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__STRUCT_H_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/DetectionBox in the package tensorrt_detect_msgs.
typedef struct tensorrt_detect_msgs__msg__DetectionBox
{
  int32_t idx;
  float confidence;
  int32_t x;
  int32_t y;
  int32_t width;
  int32_t height;
  int32_t armor_color;
  int32_t car_x;
  int32_t car_y;
  int32_t car_width;
  int32_t car_height;
  float world_x;
  float world_y;
  float fps;
} tensorrt_detect_msgs__msg__DetectionBox;

// Struct for a sequence of tensorrt_detect_msgs__msg__DetectionBox.
typedef struct tensorrt_detect_msgs__msg__DetectionBox__Sequence
{
  tensorrt_detect_msgs__msg__DetectionBox * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tensorrt_detect_msgs__msg__DetectionBox__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__STRUCT_H_
