// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/radar_map.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__BUILDER_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tensorrt_detect_msgs/msg/detail/radar_map__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tensorrt_detect_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarMap_red_y
{
public:
  explicit Init_RadarMap_red_y(::tensorrt_detect_msgs::msg::RadarMap & msg)
  : msg_(msg)
  {}
  ::tensorrt_detect_msgs::msg::RadarMap red_y(::tensorrt_detect_msgs::msg::RadarMap::_red_y_type arg)
  {
    msg_.red_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::RadarMap msg_;
};

class Init_RadarMap_red_x
{
public:
  explicit Init_RadarMap_red_x(::tensorrt_detect_msgs::msg::RadarMap & msg)
  : msg_(msg)
  {}
  Init_RadarMap_red_y red_x(::tensorrt_detect_msgs::msg::RadarMap::_red_x_type arg)
  {
    msg_.red_x = std::move(arg);
    return Init_RadarMap_red_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::RadarMap msg_;
};

class Init_RadarMap_blue_y
{
public:
  explicit Init_RadarMap_blue_y(::tensorrt_detect_msgs::msg::RadarMap & msg)
  : msg_(msg)
  {}
  Init_RadarMap_red_x blue_y(::tensorrt_detect_msgs::msg::RadarMap::_blue_y_type arg)
  {
    msg_.blue_y = std::move(arg);
    return Init_RadarMap_red_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::RadarMap msg_;
};

class Init_RadarMap_blue_x
{
public:
  explicit Init_RadarMap_blue_x(::tensorrt_detect_msgs::msg::RadarMap & msg)
  : msg_(msg)
  {}
  Init_RadarMap_blue_y blue_x(::tensorrt_detect_msgs::msg::RadarMap::_blue_x_type arg)
  {
    msg_.blue_x = std::move(arg);
    return Init_RadarMap_blue_y(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::RadarMap msg_;
};

class Init_RadarMap_header
{
public:
  Init_RadarMap_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarMap_blue_x header(::tensorrt_detect_msgs::msg::RadarMap::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadarMap_blue_x(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::RadarMap msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tensorrt_detect_msgs::msg::RadarMap>()
{
  return tensorrt_detect_msgs::msg::builder::Init_RadarMap_header();
}

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__BUILDER_HPP_
