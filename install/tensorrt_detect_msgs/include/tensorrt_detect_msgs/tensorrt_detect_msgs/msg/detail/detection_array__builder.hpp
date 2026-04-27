// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tensorrt_detect_msgs:msg/DetectionArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/detection_array.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tensorrt_detect_msgs/msg/detail/detection_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tensorrt_detect_msgs
{

namespace msg
{

namespace builder
{

class Init_DetectionArray_detections
{
public:
  explicit Init_DetectionArray_detections(::tensorrt_detect_msgs::msg::DetectionArray & msg)
  : msg_(msg)
  {}
  ::tensorrt_detect_msgs::msg::DetectionArray detections(::tensorrt_detect_msgs::msg::DetectionArray::_detections_type arg)
  {
    msg_.detections = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionArray msg_;
};

class Init_DetectionArray_header
{
public:
  Init_DetectionArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DetectionArray_detections header(::tensorrt_detect_msgs::msg::DetectionArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DetectionArray_detections(msg_);
  }

private:
  ::tensorrt_detect_msgs::msg::DetectionArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tensorrt_detect_msgs::msg::DetectionArray>()
{
  return tensorrt_detect_msgs::msg::builder::Init_DetectionArray_header();
}

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_ARRAY__BUILDER_HPP_
