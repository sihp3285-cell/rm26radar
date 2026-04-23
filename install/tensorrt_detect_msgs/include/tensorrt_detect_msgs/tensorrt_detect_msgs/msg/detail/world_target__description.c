// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tensorrt_detect_msgs:msg/WorldTarget.idl
// generated code does not contain a copyright notice

#include "tensorrt_detect_msgs/msg/detail/world_target__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tensorrt_detect_msgs
const rosidl_type_hash_t *
tensorrt_detect_msgs__msg__WorldTarget__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xd0, 0x94, 0x27, 0x87, 0x7a, 0x72, 0x92, 0x1e,
      0x26, 0x40, 0xc5, 0xfc, 0xcd, 0xb8, 0xc6, 0xaa,
      0x40, 0xc4, 0xb1, 0x62, 0x2e, 0x27, 0xf1, 0x68,
      0x0f, 0x8b, 0x52, 0xcf, 0x2c, 0xbb, 0xad, 0x23,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char tensorrt_detect_msgs__msg__WorldTarget__TYPE_NAME[] = "tensorrt_detect_msgs/msg/WorldTarget";

// Define type names, field names, and default values
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__idx[] = "idx";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__class_id[] = "class_id";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__score[] = "score";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__valid[] = "valid";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__world_x[] = "world_x";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__world_y[] = "world_y";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__world_z[] = "world_z";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_x[] = "bbox_x";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_y[] = "bbox_y";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_w[] = "bbox_w";
static char tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_h[] = "bbox_h";

static rosidl_runtime_c__type_description__Field tensorrt_detect_msgs__msg__WorldTarget__FIELDS[] = {
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__idx, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__class_id, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__score, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__valid, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__world_x, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__world_y, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__world_z, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_x, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_y, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_w, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__WorldTarget__FIELD_NAME__bbox_h, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tensorrt_detect_msgs__msg__WorldTarget__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tensorrt_detect_msgs__msg__WorldTarget__TYPE_NAME, 36, 36},
      {tensorrt_detect_msgs__msg__WorldTarget__FIELDS, 11, 11},
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
  "int32 class_id\n"
  "float32 score\n"
  "\n"
  "bool valid\n"
  "\n"
  "float32 world_x\n"
  "float32 world_y\n"
  "float32 world_z\n"
  "\n"
  "int32 bbox_x\n"
  "int32 bbox_y\n"
  "int32 bbox_w\n"
  "int32 bbox_h";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tensorrt_detect_msgs__msg__WorldTarget__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tensorrt_detect_msgs__msg__WorldTarget__TYPE_NAME, 36, 36},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 153, 153},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tensorrt_detect_msgs__msg__WorldTarget__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tensorrt_detect_msgs__msg__WorldTarget__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
