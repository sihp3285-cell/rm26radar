// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tensorrt_detect_msgs:msg/DetectionBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/detection_box.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__BUILDER_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tensorrt_detect_msgs/msg/detail/detection_box__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tensorrt_detect_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionBox_fps
{
public:
  explicit Init_DetectionBox_fps(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  ::tensorrt_detect_msgs::msg::DetectionBox fps(::tensorrt_detect_msgs::msg::DetectionBox::_fps_type arg)
  {
    msg_.fps = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_world_y
{
public:
  explicit Init_DetectionBox_world_y(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_fps world_y(::tensorrt_detect_msgs::msg::DetectionBox::_world_y_type arg)
  {
    msg_.world_y = std::move(arg);
    return Init_DetectionBox_fps(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_world_x
{
public:
  explicit Init_DetectionBox_world_x(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_world_y world_x(::tensorrt_detect_msgs::msg::DetectionBox::_world_x_type arg)
  {
    msg_.world_x = std::move(arg);
    return Init_DetectionBox_world_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_car_height
{
public:
  explicit Init_DetectionBox_car_height(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_world_x car_height(::tensorrt_detect_msgs::msg::DetectionBox::_car_height_type arg)
  {
    msg_.car_height = std::move(arg);
    return Init_DetectionBox_world_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_car_width
{
public:
  explicit Init_DetectionBox_car_width(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_car_height car_width(::tensorrt_detect_msgs::msg::DetectionBox::_car_width_type arg)
  {
    msg_.car_width = std::move(arg);
    return Init_DetectionBox_car_height(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_car_y
{
public:
  explicit Init_DetectionBox_car_y(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_car_width car_y(::tensorrt_detect_msgs::msg::DetectionBox::_car_y_type arg)
  {
    msg_.car_y = std::move(arg);
    return Init_DetectionBox_car_width(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_car_x
{
public:
  explicit Init_DetectionBox_car_x(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_car_y car_x(::tensorrt_detect_msgs::msg::DetectionBox::_car_x_type arg)
  {
    msg_.car_x = std::move(arg);
    return Init_DetectionBox_car_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_armor_color
{
public:
  explicit Init_DetectionBox_armor_color(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_car_x armor_color(::tensorrt_detect_msgs::msg::DetectionBox::_armor_color_type arg)
  {
    msg_.armor_color = std::move(arg);
    return Init_DetectionBox_car_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_height
{
public:
  explicit Init_DetectionBox_height(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_armor_color height(::tensorrt_detect_msgs::msg::DetectionBox::_height_type arg)
  {
    msg_.height = std::move(arg);
    return Init_DetectionBox_armor_color(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_width
{
public:
  explicit Init_DetectionBox_width(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_height width(::tensorrt_detect_msgs::msg::DetectionBox::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_DetectionBox_height(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_y
{
public:
  explicit Init_DetectionBox_y(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_width y(::tensorrt_detect_msgs::msg::DetectionBox::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_DetectionBox_width(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_x
{
public:
  explicit Init_DetectionBox_x(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_y x(::tensorrt_detect_msgs::msg::DetectionBox::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_DetectionBox_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_confidence
{
public:
  explicit Init_DetectionBox_confidence(::tensorrt_detect_msgs::msg::DetectionBox & msg)
  : msg_(msg)
  {}
  Init_DetectionBox_x confidence(::tensorrt_detect_msgs::msg::DetectionBox::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_DetectionBox_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

class Init_DetectionBox_idx
{
public:
  Init_DetectionBox_idx()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionBox_confidence idx(::tensorrt_detect_msgs::msg::DetectionBox::_idx_type arg)
  {
    msg_.idx = std::move(arg);
    return Init_DetectionBox_confidence(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionBox msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tensorrt_detect_msgs::msg::DetectionBox>()
{
  return tensorrt_detect_msgs::msg::builder::Init_DetectionBox_idx();
}

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__BUILDER_HPP_
