// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tensorrt_detect_msgs:msg/DetectionBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/detection_box.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__TRAITS_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tensorrt_detect_msgs/msg/detail/detection_box__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tensorrt_detect_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DetectionBox & msg,
  std::ostream & out)
{
  out << "{";
  // member: idx
  {
    out << "idx: ";
    rosidl_generator_traits::value_to_yaml(msg.idx, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: armor_color
  {
    out << "armor_color: ";
    rosidl_generator_traits::value_to_yaml(msg.armor_color, out);
    out << ", ";
  }

  // member: car_x
  {
    out << "car_x: ";
    rosidl_generator_traits::value_to_yaml(msg.car_x, out);
    out << ", ";
  }

  // member: car_y
  {
    out << "car_y: ";
    rosidl_generator_traits::value_to_yaml(msg.car_y, out);
    out << ", ";
  }

  // member: car_width
  {
    out << "car_width: ";
    rosidl_generator_traits::value_to_yaml(msg.car_width, out);
    out << ", ";
  }

  // member: car_height
  {
    out << "car_height: ";
    rosidl_generator_traits::value_to_yaml(msg.car_height, out);
    out << ", ";
  }

  // member: world_x
  {
    out << "world_x: ";
    rosidl_generator_traits::value_to_yaml(msg.world_x, out);
    out << ", ";
  }

  // member: world_y
  {
    out << "world_y: ";
    rosidl_generator_traits::value_to_yaml(msg.world_y, out);
    out << ", ";
  }

  // member: fps
  {
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DetectionBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: idx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "idx: ";
    rosidl_generator_traits::value_to_yaml(msg.idx, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: armor_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "armor_color: ";
    rosidl_generator_traits::value_to_yaml(msg.armor_color, out);
    out << "\n";
  }

  // member: car_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "car_x: ";
    rosidl_generator_traits::value_to_yaml(msg.car_x, out);
    out << "\n";
  }

  // member: car_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "car_y: ";
    rosidl_generator_traits::value_to_yaml(msg.car_y, out);
    out << "\n";
  }

  // member: car_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "car_width: ";
    rosidl_generator_traits::value_to_yaml(msg.car_width, out);
    out << "\n";
  }

  // member: car_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "car_height: ";
    rosidl_generator_traits::value_to_yaml(msg.car_height, out);
    out << "\n";
  }

  // member: world_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_x: ";
    rosidl_generator_traits::value_to_yaml(msg.world_x, out);
    out << "\n";
  }

  // member: world_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_y: ";
    rosidl_generator_traits::value_to_yaml(msg.world_y, out);
    out << "\n";
  }

  // member: fps
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fps: ";
    rosidl_generator_traits::value_to_yaml(msg.fps, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DetectionBox & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tensorrt_detect_msgs

namespace rosidl_generator_traits
{

[[deprecated("use tensorrt_detect_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tensorrt_detect_msgs::msg::DetectionBox & msg,
  std::ostream & out, size_t indentation = 0)
{
  tensorrt_detect_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tensorrt_detect_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tensorrt_detect_msgs::msg::DetectionBox & msg)
{
  return tensorrt_detect_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tensorrt_detect_msgs::msg::DetectionBox>()
{
  return "tensorrt_detect_msgs::msg::DetectionBox";
}

template<>
inline const char * name<tensorrt_detect_msgs::msg::DetectionBox>()
{
  return "tensorrt_detect_msgs/msg/DetectionBox";
}

template<>
struct has_fixed_size<tensorrt_detect_msgs::msg::DetectionBox>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tensorrt_detect_msgs::msg::DetectionBox>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tensorrt_detect_msgs::msg::DetectionBox>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__TRAITS_HPP_
