// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tensorrt_detect_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/detection_array.h"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_

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
// Member 'detections'
#include "tensorrt_detect_msgs/msg/detail/detection_box__struct.h"

/// Struct defined in msg/DetectionArray in the package tensorrt_detect_msgs.
typedef struct tensorrt_detect_msgs__msg__DetectionArray
{
  std_msgs__msg__Header header;
  tensorrt_detect_msgs__msg__DetectionBox__Sequence detections;
} tensorrt_detect_msgs__msg__DetectionArray;

// Struct for a sequence of tensorrt_detect_msgs__msg__DetectionArray.
typedef struct tensorrt_detect_msgs__msg__DetectionArray__Sequence
{
  tensorrt_detect_msgs__msg__DetectionArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tensorrt_detect_msgs__msg__DetectionArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__STRUCT_H_
