// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target_array.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__BUILDER_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tensorrt_detect_msgs/msg/detail/world_target_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tensorrt_detect_msgs
{

namespace msg
{

namespace builder
{

class Init_WorldTargetArray_targets
{
public:
  explicit Init_WorldTargetArray_targets(::tensorrt_detect_msgs::msg::WorldTargetArray & msg)
  : msg_(msg)
  {}
  ::tensorrt_detect_msgs::msg::WorldTargetArray targets(::tensorrt_detect_msgs::msg::WorldTargetArray::_targets_type arg)
  {
    msg_.targets = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTargetArray msg_;
};

class Init_WorldTargetArray_header
{
public:
  Init_WorldTargetArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorldTargetArray_targets header(::tensorrt_detect_msgs::msg::WorldTargetArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_WorldTargetArray_targets(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTargetArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tensorrt_detect_msgs::msg::WorldTargetArray>()
{
  return tensorrt_detect_msgs::msg::builder::Init_WorldTargetArray_header();
}

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__BUILDER_HPP_
