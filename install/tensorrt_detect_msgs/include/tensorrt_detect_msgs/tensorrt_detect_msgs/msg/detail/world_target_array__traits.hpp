// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target_array.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__TRAITS_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tensorrt_detect_msgs/msg/detail/world_target_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'targets'
#include "tensorrt_detect_msgs/msg/detail/world_target__traits.hpp"

namespace tensorrt_detect_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorldTargetArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: targets
  {
    if (msg.targets.size() == 0) {
      out << "targets: []";
    } else {
      out << "targets: [";
      size_t pending_items = msg.targets.size();
      for (auto item : msg.targets) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WorldTargetArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: targets
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.targets.size() == 0) {
      out << "targets: []\n";
    } else {
      out << "targets:\n";
      for (auto item : msg.targets) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorldTargetArray & msg, bool use_flow_style = false)
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
  const tensorrt_detect_msgs::msg::WorldTargetArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  tensorrt_detect_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tensorrt_detect_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tensorrt_detect_msgs::msg::WorldTargetArray & msg)
{
  return tensorrt_detect_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tensorrt_detect_msgs::msg::WorldTargetArray>()
{
  return "tensorrt_detect_msgs::msg::WorldTargetArray";
}

template<>
inline const char * name<tensorrt_detect_msgs::msg::WorldTargetArray>()
{
  return "tensorrt_detect_msgs/msg/WorldTargetArray";
}

template<>
struct has_fixed_size<tensorrt_detect_msgs::msg::WorldTargetArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tensorrt_detect_msgs::msg::WorldTargetArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tensorrt_detect_msgs::msg::WorldTargetArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__TRAITS_HPP_
