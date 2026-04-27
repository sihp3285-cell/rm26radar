// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/radar_map.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__TRAITS_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tensorrt_detect_msgs/msg/detail/radar_map__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tensorrt_detect_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadarMap & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: blue_x
  {
    if (msg.blue_x.size() == 0) {
      out << "blue_x: []";
    } else {
      out << "blue_x: [";
      size_t pending_items = msg.blue_x.size();
      for (auto item : msg.blue_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: blue_y
  {
    if (msg.blue_y.size() == 0) {
      out << "blue_y: []";
    } else {
      out << "blue_y: [";
      size_t pending_items = msg.blue_y.size();
      for (auto item : msg.blue_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: red_x
  {
    if (msg.red_x.size() == 0) {
      out << "red_x: []";
    } else {
      out << "red_x: [";
      size_t pending_items = msg.red_x.size();
      for (auto item : msg.red_x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: red_y
  {
    if (msg.red_y.size() == 0) {
      out << "red_y: []";
    } else {
      out << "red_y: [";
      size_t pending_items = msg.red_y.size();
      for (auto item : msg.red_y) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const RadarMap & msg,
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

  // member: blue_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.blue_x.size() == 0) {
      out << "blue_x: []\n";
    } else {
      out << "blue_x:\n";
      for (auto item : msg.blue_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: blue_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.blue_y.size() == 0) {
      out << "blue_y: []\n";
    } else {
      out << "blue_y:\n";
      for (auto item : msg.blue_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: red_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.red_x.size() == 0) {
      out << "red_x: []\n";
    } else {
      out << "red_x:\n";
      for (auto item : msg.red_x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: red_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.red_y.size() == 0) {
      out << "red_y: []\n";
    } else {
      out << "red_y:\n";
      for (auto item : msg.red_y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadarMap & msg, bool use_flow_style = false)
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
  const tensorrt_detect_msgs::msg::RadarMap & msg,
  std::ostream & out, size_t indentation = 0)
{
  tensorrt_detect_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tensorrt_detect_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const tensorrt_detect_msgs::msg::RadarMap & msg)
{
  return tensorrt_detect_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tensorrt_detect_msgs::msg::RadarMap>()
{
  return "tensorrt_detect_msgs::msg::RadarMap";
}

template<>
inline const char * name<tensorrt_detect_msgs::msg::RadarMap>()
{
  return "tensorrt_detect_msgs/msg/RadarMap";
}

template<>
struct has_fixed_size<tensorrt_detect_msgs::msg::RadarMap>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<tensorrt_detect_msgs::msg::RadarMap>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<tensorrt_detect_msgs::msg::RadarMap>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__TRAITS_HPP_
