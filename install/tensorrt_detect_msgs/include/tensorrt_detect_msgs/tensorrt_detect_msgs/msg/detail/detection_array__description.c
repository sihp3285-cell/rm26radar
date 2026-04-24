// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from tensorrt_detect_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

#include "tensorrt_detect_msgs/msg/detail/detection_array__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_tensorrt_detect_msgs
const rosidl_type_hash_t *
tensorrt_detect_msgs__msg__DetectionArray__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x19, 0x0e, 0xfd, 0xa4, 0x09, 0xa3, 0xfe, 0xc4,
      0xf6, 0x8d, 0xff, 0x1b, 0xd5, 0x40, 0x23, 0x5b,
      0x95, 0xa4, 0x07, 0x38, 0xf5, 0xdc, 0xb0, 0x55,
      0xe1, 0x35, 0x2d, 0x85, 0x58, 0x37, 0x29, 0x4d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "std_msgs/msg/detail/header__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "tensorrt_detect_msgs/msg/detail/detection_box__functions.h"

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
static const rosidl_type_hash_t tensorrt_detect_msgs__msg__DetectionBox__EXPECTED_HASH = {1, {
    0xae, 0x58, 0x72, 0x46, 0x5c, 0x18, 0xd4, 0x2a,
    0x26, 0x6e, 0xfd, 0xa0, 0x32, 0xef, 0xdb, 0xdb,
    0x48, 0x4a, 0x55, 0x77, 0xd0, 0xfd, 0x33, 0xb1,
    0xd3, 0x0e, 0xfb, 0x79, 0xc2, 0x40, 0x35, 0x37,
  }};
#endif

static char tensorrt_detect_msgs__msg__DetectionArray__TYPE_NAME[] = "tensorrt_detect_msgs/msg/DetectionArray";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char std_msgs__msg__Header__TYPE_NAME[] = "std_msgs/msg/Header";
static char tensorrt_detect_msgs__msg__DetectionBox__TYPE_NAME[] = "tensorrt_detect_msgs/msg/DetectionBox";

// Define type names, field names, and default values
static char tensorrt_detect_msgs__msg__DetectionArray__FIELD_NAME__header[] = "header";
static char tensorrt_detect_msgs__msg__DetectionArray__FIELD_NAME__detections[] = "detections";

static rosidl_runtime_c__type_description__Field tensorrt_detect_msgs__msg__DetectionArray__FIELDS[] = {
  {
    {tensorrt_detect_msgs__msg__DetectionArray__FIELD_NAME__header, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    },
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionArray__FIELD_NAME__detections, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {tensorrt_detect_msgs__msg__DetectionBox__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription tensorrt_detect_msgs__msg__DetectionArray__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {std_msgs__msg__Header__TYPE_NAME, 19, 19},
    {NULL, 0, 0},
  },
  {
    {tensorrt_detect_msgs__msg__DetectionBox__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
tensorrt_detect_msgs__msg__DetectionArray__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {tensorrt_detect_msgs__msg__DetectionArray__TYPE_NAME, 39, 39},
      {tensorrt_detect_msgs__msg__DetectionArray__FIELDS, 2, 2},
    },
    {tensorrt_detect_msgs__msg__DetectionArray__REFERENCED_TYPE_DESCRIPTIONS, 3, 3},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&std_msgs__msg__Header__EXPECTED_HASH, std_msgs__msg__Header__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = std_msgs__msg__Header__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&tensorrt_detect_msgs__msg__DetectionBox__EXPECTED_HASH, tensorrt_detect_msgs__msg__DetectionBox__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[2].fields = tensorrt_detect_msgs__msg__DetectionBox__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "std_msgs/Header header\n"
  "DetectionBox[] detections";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
tensorrt_detect_msgs__msg__DetectionArray__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {tensorrt_detect_msgs__msg__DetectionArray__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 49, 49},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
tensorrt_detect_msgs__msg__DetectionArray__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[4];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 4, 4};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *tensorrt_detect_msgs__msg__DetectionArray__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *std_msgs__msg__Header__get_individual_type_description_source(NULL);
    sources[3] = *tensorrt_detect_msgs__msg__DetectionBox__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
