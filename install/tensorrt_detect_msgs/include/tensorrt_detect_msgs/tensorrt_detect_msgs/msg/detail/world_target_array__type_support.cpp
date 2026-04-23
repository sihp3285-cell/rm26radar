// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "tensorrt_detect_msgs/msg/detail/world_target_array__functions.h"
#include "tensorrt_detect_msgs/msg/detail/world_target_array__struct.hpp"
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

void WorldTargetArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) tensorrt_detect_msgs::msg::WorldTargetArray(_init);
}

void WorldTargetArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<tensorrt_detect_msgs::msg::WorldTargetArray *>(message_memory);
  typed_message->~WorldTargetArray();
}

size_t size_function__WorldTargetArray__targets(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<tensorrt_detect_msgs::msg::WorldTarget> *>(untyped_member);
  return member->size();
}

const void * get_const_function__WorldTargetArray__targets(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<tensorrt_detect_msgs::msg::WorldTarget> *>(untyped_member);
  return &member[index];
}

void * get_function__WorldTargetArray__targets(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<tensorrt_detect_msgs::msg::WorldTarget> *>(untyped_member);
  return &member[index];
}

void fetch_function__WorldTargetArray__targets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const tensorrt_detect_msgs::msg::WorldTarget *>(
    get_const_function__WorldTargetArray__targets(untyped_member, index));
  auto & value = *reinterpret_cast<tensorrt_detect_msgs::msg::WorldTarget *>(untyped_value);
  value = item;
}

void assign_function__WorldTargetArray__targets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<tensorrt_detect_msgs::msg::WorldTarget *>(
    get_function__WorldTargetArray__targets(untyped_member, index));
  const auto & value = *reinterpret_cast<const tensorrt_detect_msgs::msg::WorldTarget *>(untyped_value);
  item = value;
}

void resize_function__WorldTargetArray__targets(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<tensorrt_detect_msgs::msg::WorldTarget> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember WorldTargetArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::WorldTargetArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "targets",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<tensorrt_detect_msgs::msg::WorldTarget>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tensorrt_detect_msgs::msg::WorldTargetArray, targets),  // bytes offset in struct
    nullptr,  // default value
    size_function__WorldTargetArray__targets,  // size() function pointer
    get_const_function__WorldTargetArray__targets,  // get_const(index) function pointer
    get_function__WorldTargetArray__targets,  // get(index) function pointer
    fetch_function__WorldTargetArray__targets,  // fetch(index, &value) function pointer
    assign_function__WorldTargetArray__targets,  // assign(index, value) function pointer
    resize_function__WorldTargetArray__targets  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers WorldTargetArray_message_members = {
  "tensorrt_detect_msgs::msg",  // message namespace
  "WorldTargetArray",  // message name
  2,  // number of fields
  sizeof(tensorrt_detect_msgs::msg::WorldTargetArray),
  false,  // has_any_key_member_
  WorldTargetArray_message_member_array,  // message members
  WorldTargetArray_init_function,  // function to initialize message memory (memory has to be allocated)
  WorldTargetArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t WorldTargetArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &WorldTargetArray_message_members,
  get_message_typesupport_handle_function,
  &tensorrt_detect_msgs__msg__WorldTargetArray__get_type_hash,
  &tensorrt_detect_msgs__msg__WorldTargetArray__get_type_description,
  &tensorrt_detect_msgs__msg__WorldTargetArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace tensorrt_detect_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<tensorrt_detect_msgs::msg::WorldTargetArray>()
{
  return &::tensorrt_detect_msgs::msg::rosidl_typesupport_introspection_cpp::WorldTargetArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, tensorrt_detect_msgs, msg, WorldTargetArray)() {
  return &::tensorrt_detect_msgs::msg::rosidl_typesupport_introspection_cpp::WorldTargetArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
