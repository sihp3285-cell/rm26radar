// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tensorrt_detect_msgs:msg/DetectionBox.idl
// generated code does not contain a copyright notice

#include "tensorrt_detect_msgs/msg/detail/detection_box__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tensorrt_detect_msgs
const rosidl_type_hash_t *
tensorrt_detect_msgs__msg__DetectionBox__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xae, 0x58, 0x72, 0x46, 0x5c, 0x18, 0xd4, 0x2a,
      0x26, 0x6e, 0xfd, 0xa0, 0x32, 0xef, 0xdb, 0xdb,
      0x48, 0x4a, 0x55, 0x77, 0xd0, 0xfd, 0x33, 0xb1,
      0xd3, 0x0e, 0xfb, 0x79, 0xc2, 0x40, 0x35, 0x37,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char tensorrt_detect_msgs__msg__DetectionBox__TYPE_NAME[] = "tensorrt_detect_msgs/msg/DetectionBox";

// Define type names, field names, and default values
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__idx[] = "idx";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__confidence[] = "confidence";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__x[] = "x";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__y[] = "y";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__width[] = "width";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__height[] = "height";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__armor_color[] = "armor_color";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_x[] = "car_x";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_y[] = "car_y";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_width[] = "car_width";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_height[] = "car_height";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__world_x[] = "world_x";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__world_y[] = "world_y";
static char tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__fps[] = "fps";

static rosidl_runtime_c__type_description__Field tensorrt_detect_msgs__msg__DetectionBox__FIELDS[] = {
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__idx, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__confidence, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__width, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__height, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__armor_color, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_width, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__car_height, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__world_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__world_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__FIELD_NAME__fps, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tensorrt_detect_msgs__msg__DetectionBox__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tensorrt_detect_msgs__msg__DetectionBox__TYPE_NAME, 37, 37},
      {tensorrt_detect_msgs__msg__DetectionBox__FIELDS, 14, 14},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 idx\n"
  "float32 confidence\n"
  "int32 x\n"
  "int32 y\n"
  "int32 width\n"
  "int32 height\n"
  "int32 armor_color\n"
  "int32 car_x\n"
  "int32 car_y\n"
  "int32 car_width\n"
  "int32 car_height\n"
  "float32 world_x\n"
  "float32 world_y\n"
  "float32 fps";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tensorrt_detect_msgs__msg__DetectionBox__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tensorrt_detect_msgs__msg__DetectionBox__TYPE_NAME, 37, 37},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 189, 189},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tensorrt_detect_msgs__msg__DetectionBox__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tensorrt_detect_msgs__msg__DetectionBox__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
