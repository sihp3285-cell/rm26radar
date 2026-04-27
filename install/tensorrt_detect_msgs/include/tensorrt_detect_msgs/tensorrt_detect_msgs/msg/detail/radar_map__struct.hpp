// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tensorrt_detect_msgs:msg/RadarMap.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/radar_map.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__STRUCT_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tensorrt_detect_msgs__msg__RadarMap __attribute__((deprecated))
#else
# define DEPRECATED__tensorrt_detect_msgs__msg__RadarMap __declspec(deprecated)
#endif

namespace tensorrt_detect_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarMap_
{
  using Type = RadarMap_<ContainerAllocator>;

  explicit RadarMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 6>::iterator, float>(this->blue_x.begin(), this->blue_x.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->blue_y.begin(), this->blue_y.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->red_x.begin(), this->red_x.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->red_y.begin(), this->red_y.end(), 0.0f);
    }
  }

  explicit RadarMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    blue_x(_alloc),
    blue_y(_alloc),
    red_x(_alloc),
    red_y(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 6>::iterator, float>(this->blue_x.begin(), this->blue_x.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->blue_y.begin(), this->blue_y.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->red_x.begin(), this->red_x.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->red_y.begin(), this->red_y.end(), 0.0f);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _blue_x_type =
    std::array<float, 6>;
  _blue_x_type blue_x;
  using _blue_y_type =
    std::array<float, 6>;
  _blue_y_type blue_y;
  using _red_x_type =
    std::array<float, 6>;
  _red_x_type red_x;
  using _red_y_type =
    std::array<float, 6>;
  _red_y_type red_y;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__blue_x(
    const std::array<float, 6> & _arg)
  {
    this->blue_x = _arg;
    return *this;
  }
  Type & set__blue_y(
    const std::array<float, 6> & _arg)
  {
    this->blue_y = _arg;
    return *this;
  }
  Type & set__red_x(
    const std::array<float, 6> & _arg)
  {
    this->red_x = _arg;
    return *this;
  }
  Type & set__red_y(
    const std::array<float, 6> & _arg)
  {
    this->red_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tensorrt_detect_msgs__msg__RadarMap
    std::shared_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tensorrt_detect_msgs__msg__RadarMap
    std::shared_ptr<tensorrt_detect_msgs::msg::RadarMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarMap_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->blue_x != other.blue_x) {
      return false;
    }
    if (this->blue_y != other.blue_y) {
      return false;
    }
    if (this->red_x != other.red_x) {
      return false;
    }
    if (this->red_y != other.red_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarMap_

// alias to use template instance with default allocator
using RadarMap =
  tensorrt_detect_msgs::msg::RadarMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__RADAR_MAP__STRUCT_HPP_
