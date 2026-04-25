// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tensorrt_detect_msgs/msg/detail/radar_map__functions.h"
#include "tensorrt_detect_msgs/msg/detail/radar_map__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace tensorrt_detect_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void RadarMap_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tensorrt_detect_msgs::msg::RadarMap(_init);
}

void RadarMap_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tensorrt_detect_msgs::msg::RadarMap *>(message_memory);
  typed_message->~RadarMap();
}

size_t size_function__RadarMap__blue_x(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RadarMap__blue_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarMap__blue_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarMap__blue_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarMap__blue_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarMap__blue_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarMap__blue_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__RadarMap__blue_y(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RadarMap__blue_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarMap__blue_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarMap__blue_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarMap__blue_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarMap__blue_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarMap__blue_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__RadarMap__red_x(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RadarMap__red_x(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarMap__red_x(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarMap__red_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarMap__red_x(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarMap__red_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarMap__red_x(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

size_t size_function__RadarMap__red_y(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__RadarMap__red_y(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__RadarMap__red_y(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__RadarMap__red_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__RadarMap__red_y(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__RadarMap__red_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__RadarMap__red_y(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember RadarMap_message_member_array[5] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::RadarMap, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "blue_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::RadarMap, blue_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarMap__blue_x,  // size() function pointer
    get_const_function__RadarMap__blue_x,  // get_const(index) function pointer
    get_function__RadarMap__blue_x,  // get(index) function pointer
    fetch_function__RadarMap__blue_x,  // fetch(index, &value) function pointer
    assign_function__RadarMap__blue_x,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "blue_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::RadarMap, blue_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarMap__blue_y,  // size() function pointer
    get_const_function__RadarMap__blue_y,  // get_const(index) function pointer
    get_function__RadarMap__blue_y,  // get(index) function pointer
    fetch_function__RadarMap__blue_y,  // fetch(index, &value) function pointer
    assign_function__RadarMap__blue_y,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "red_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::RadarMap, red_x),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarMap__red_x,  // size() function pointer
    get_const_function__RadarMap__red_x,  // get_const(index) function pointer
    get_function__RadarMap__red_x,  // get(index) function pointer
    fetch_function__RadarMap__red_x,  // fetch(index, &value) function pointer
    assign_function__RadarMap__red_x,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "red_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::RadarMap, red_y),  // bytes offset in struct
    nullptr,  // default value
    size_function__RadarMap__red_y,  // size() function pointer
    get_const_function__RadarMap__red_y,  // get_const(index) function pointer
    get_function__RadarMap__red_y,  // get(index) function pointer
    fetch_function__RadarMap__red_y,  // fetch(index, &value) function pointer
    assign_function__RadarMap__red_y,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers RadarMap_message_members = {
  "tensorrt_detect_msgs::msg",  // message namespace
  "RadarMap",  // message name
  5,  // number of fields
  sizeof(tensorrt_detect_msgs::msg::RadarMap),
  false,  // has_any_key_member_
  RadarMap_message_member_array,  // message members
  RadarMap_init_function,  // function to initialize message memory (memory has to be allocated)
  RadarMap_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t RadarMap_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &RadarMap_message_members,
  get_message_typesupport_handle_function,
  &tensorrt_detect_msgs__msg__RadarMap__get_type_hash,
  &tensorrt_detect_msgs__msg__RadarMap__get_type_description,
  &tensorrt_detect_msgs__msg__RadarMap__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tensorrt_detect_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tensorrt_detect_msgs::msg::RadarMap>()
{
  return &::tensorrt_detect_msgs::msg::rosidl_typesupport_introspection_cpp::RadarMap_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tensorrt_detect_msgs, msg, RadarMap)() {
  return &::tensorrt_detect_msgs::msg::rosidl_typesupport_introspection_cpp::RadarMap_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
