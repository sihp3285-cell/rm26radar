// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tensorrt_detect_msgs:msg/WorldTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__BUILDER_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tensorrt_detect_msgs/msg/detail/world_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tensorrt_detect_msgs
{

namespace msg
{

namespace builder
{

class Init_WorldTarget_bbox_h
{
public:
  explicit Init_WorldTarget_bbox_h(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  ::tensorrt_detect_msgs::msg::WorldTarget bbox_h(::tensorrt_detect_msgs::msg::WorldTarget::_bbox_h_type arg)
  {
    msg_.bbox_h = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_bbox_w
{
public:
  explicit Init_WorldTarget_bbox_w(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_bbox_h bbox_w(::tensorrt_detect_msgs::msg::WorldTarget::_bbox_w_type arg)
  {
    msg_.bbox_w = std::move(arg);
    return Init_WorldTarget_bbox_h(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_bbox_y
{
public:
  explicit Init_WorldTarget_bbox_y(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_bbox_w bbox_y(::tensorrt_detect_msgs::msg::WorldTarget::_bbox_y_type arg)
  {
    msg_.bbox_y = std::move(arg);
    return Init_WorldTarget_bbox_w(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_bbox_x
{
public:
  explicit Init_WorldTarget_bbox_x(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_bbox_y bbox_x(::tensorrt_detect_msgs::msg::WorldTarget::_bbox_x_type arg)
  {
    msg_.bbox_x = std::move(arg);
    return Init_WorldTarget_bbox_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_world_z
{
public:
  explicit Init_WorldTarget_world_z(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_bbox_x world_z(::tensorrt_detect_msgs::msg::WorldTarget::_world_z_type arg)
  {
    msg_.world_z = std::move(arg);
    return Init_WorldTarget_bbox_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_world_y
{
public:
  explicit Init_WorldTarget_world_y(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_world_z world_y(::tensorrt_detect_msgs::msg::WorldTarget::_world_y_type arg)
  {
    msg_.world_y = std::move(arg);
    return Init_WorldTarget_world_z(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_world_x
{
public:
  explicit Init_WorldTarget_world_x(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_world_y world_x(::tensorrt_detect_msgs::msg::WorldTarget::_world_x_type arg)
  {
    msg_.world_x = std::move(arg);
    return Init_WorldTarget_world_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_valid
{
public:
  explicit Init_WorldTarget_valid(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_world_x valid(::tensorrt_detect_msgs::msg::WorldTarget::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_WorldTarget_world_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_score
{
public:
  explicit Init_WorldTarget_score(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_valid score(::tensorrt_detect_msgs::msg::WorldTarget::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_WorldTarget_valid(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_class_id
{
public:
  explicit Init_WorldTarget_class_id(::tensorrt_detect_msgs::msg::WorldTarget & msg)
  : msg_(msg)
  {}
  Init_WorldTarget_score class_id(::tensorrt_detect_msgs::msg::WorldTarget::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_WorldTarget_score(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

class Init_WorldTarget_idx
{
public:
  Init_WorldTarget_idx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorldTarget_class_id idx(::tensorrt_detect_msgs::msg::WorldTarget::_idx_type arg)
  {
    msg_.idx = std::move(arg);
    return Init_WorldTarget_class_id(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::WorldTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tensorrt_detect_msgs::msg::WorldTarget>()
{
  return tensorrt_detect_msgs::msg::builder::Init_WorldTarget_idx();
}

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__BUILDER_HPP_
