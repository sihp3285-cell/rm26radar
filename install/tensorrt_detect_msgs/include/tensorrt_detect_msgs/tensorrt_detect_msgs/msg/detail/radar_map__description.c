// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice

#include "tensorrt_detect_msgs/msg/detail/radar_map__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tensorrt_detect_msgs
const rosidl_type_hash_t *
tensorrt_detect_msgs__msg__RadarMap__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x25, 0x0b, 0xfe, 0xfc, 0xcb, 0xbc, 0x17, 0xd6,
      0xf3, 0x5b, 0xc3, 0x3b, 0x9e, 0x20, 0xe1, 0x5f,
      0x54, 0x7d, 0x3d, 0x84, 0x44, 0xe4, 0x08, 0xab,
      0x96, 0xdc, 0x1c, 0x6b, 0xc3, 0x74, 0x5c, 0x42,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "std_msgs/msg/detail/header__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t std_msgs__msg__Header__EXPECTED_HASH = {1, {
    0xf4, 0x9f, 0xb3, 0xae, 0x2c, 0xf0, 0x70, 0xf7,
    0x93, 0x64, 0x5f, 0xf7, 0x49, 0x68, 0x3a, 0xc6,
    0xb0, 0x62, 0x03, 0xe4, 0x1c, 0x89, 0x1e, 0x17,
    0x70, 0x1b, 0x1c, 0xb5, 0x97, 0xce, 0x6a, 0x01,
  }};
#endif

static char tensorrt_detect_msgs__msg__RadarMap__TYPE_NAME[] = "tensorrt_detect_msgs/msg/RadarMap";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";

// Define type names, field names, and default values
static char tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__header[] = "header";
static char tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__blue_x[] = "blue_x";
static char tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__blue_y[] = "blue_y";
static char tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__red_x[] = "red_x";
static char tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__red_y[] = "red_y";

static rosidl_runtime_c__type_description__Field tensorrt_detect_msgs__msg__RadarMap__FIELDS[] = {
  {
    {tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__blue_x, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__blue_y, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__red_x, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__RadarMap__FIELD_NAME__red_y, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      6,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription tensorrt_detect_msgs__msg__RadarMap__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tensorrt_detect_msgs__msg__RadarMap__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tensorrt_detect_msgs__msg__RadarMap__TYPE_NAME, 33, 33},
      {tensorrt_detect_msgs__msg__RadarMap__FIELDS, 5, 5},
    },
    {tensorrt_detect_msgs__msg__RadarMap__REFERENCED_TYPE_DESCRIPTIONS, 2, 2},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "\n"
  "# \\xe8\\x93\\x9d\\xe6\\x96\\xb9 1-5 \\xe5\\x8f\\xb7\\xe5\\x8f\\x8a\\xe5\\x93\\xa8\\xe5\\x85\\xb5(6)\\xe7\\x9a\\x84\\xe5\\x9c\\xb0\\xe5\\x9b\\xbe\\xe5\\x9d\\x90\\xe6\\xa0\\x87 (x, y)\n"
  "float32[6] blue_x\n"
  "float32[6] blue_y\n"
  "\n"
  "# \\xe7\\xba\\xa2\\xe6\\x96\\xb9 1-5 \\xe5\\x8f\\xb7\\xe5\\x8f\\x8a\\xe5\\x93\\xa8\\xe5\\x85\\xb5(6)\\xe7\\x9a\\x84\\xe5\\x9c\\xb0\\xe5\\x9b\\xbe\\xe5\\x9d\\x90\\xe6\\xa0\\x87 (x, y)\n"
  "float32[6] red_x\n"
  "float32[6] red_y";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tensorrt_detect_msgs__msg__RadarMap__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tensorrt_detect_msgs__msg__RadarMap__TYPE_NAME, 33, 33},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 152, 152},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tensorrt_detect_msgs__msg__RadarMap__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[3];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 3, 3};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tensorrt_detect_msgs__msg__RadarMap__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
