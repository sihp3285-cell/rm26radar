// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tensorrt_detect_msgs:msg/WorldTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__TRAITS_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tensorrt_detect_msgs/msg/detail/world_target__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tensorrt_detect_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorldTarget & msg,
  std::ostream & out)
{
  out << "{";
  // member: idx
  {
    out << "idx: ";
    rosidl_generator_traits::value_to_yaml(msg.idx, out);
    out << ", ";
  }

  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << ", ";
  }

  // member: score
  {
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << ", ";
  }

  // member: valid
  {
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
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

  // member: world_z
  {
    out << "world_z: ";
    rosidl_generator_traits::value_to_yaml(msg.world_z, out);
    out << ", ";
  }

  // member: bbox_x
  {
    out << "bbox_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x, out);
    out << ", ";
  }

  // member: bbox_y
  {
    out << "bbox_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y, out);
    out << ", ";
  }

  // member: bbox_w
  {
    out << "bbox_w: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_w, out);
    out << ", ";
  }

  // member: bbox_h
  {
    out << "bbox_h: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_h, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WorldTarget & msg,
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

  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
    out << "\n";
  }

  // member: valid
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "valid: ";
    rosidl_generator_traits::value_to_yaml(msg.valid, out);
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

  // member: world_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "world_z: ";
    rosidl_generator_traits::value_to_yaml(msg.world_z, out);
    out << "\n";
  }

  // member: bbox_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_x: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_x, out);
    out << "\n";
  }

  // member: bbox_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_y: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_y, out);
    out << "\n";
  }

  // member: bbox_w
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_w: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_w, out);
    out << "\n";
  }

  // member: bbox_h
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bbox_h: ";
    rosidl_generator_traits::value_to_yaml(msg.bbox_h, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorldTarget & msg, bool use_flow_style = false)
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
  const tensorrt_detect_msgs::msg::WorldTarget & msg,
  std::ostream & out, size_t indentation = 0)
{
  tensorrt_detect_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tensorrt_detect_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tensorrt_detect_msgs::msg::WorldTarget & msg)
{
  return tensorrt_detect_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tensorrt_detect_msgs::msg::WorldTarget>()
{
  return "tensorrt_detect_msgs::msg::WorldTarget";
}

template<>
inline const char * name<tensorrt_detect_msgs::msg::WorldTarget>()
{
  return "tensorrt_detect_msgs/msg/WorldTarget";
}

template<>
struct has_fixed_size<tensorrt_detect_msgs::msg::WorldTarget>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tensorrt_detect_msgs::msg::WorldTarget>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tensorrt_detect_msgs::msg::WorldTarget>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__TRAITS_HPP_
