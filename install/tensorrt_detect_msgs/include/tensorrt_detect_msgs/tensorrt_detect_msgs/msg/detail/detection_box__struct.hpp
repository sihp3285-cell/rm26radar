// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tensorrt_detect_msgs:msg/DetectionBox.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/detection_box.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__STRUCT_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tensorrt_detect_msgs__msg__DetectionBox __attribute__((deprecated))
#else
# define DEPRECATED__tensorrt_detect_msgs__msg__DetectionBox __declspec(deprecated)
#endif

namespace tensorrt_detect_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DetectionBox_
{
  using Type = DetectionBox_<ContainerAllocator>;

  explicit DetectionBox_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->idx = 0l;
      this->confidence = 0.0f;
      this->x = 0l;
      this->y = 0l;
      this->width = 0l;
      this->height = 0l;
      this->armor_color = 0l;
      this->car_x = 0l;
      this->car_y = 0l;
      this->car_width = 0l;
      this->car_height = 0l;
      this->world_x = 0.0f;
      this->world_y = 0.0f;
      this->fps = 0.0f;
    }
  }

  explicit DetectionBox_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->idx = 0l;
      this->confidence = 0.0f;
      this->x = 0l;
      this->y = 0l;
      this->width = 0l;
      this->height = 0l;
      this->armor_color = 0l;
      this->car_x = 0l;
      this->car_y = 0l;
      this->car_width = 0l;
      this->car_height = 0l;
      this->world_x = 0.0f;
      this->world_y = 0.0f;
      this->fps = 0.0f;
    }
  }

  // field types and members
  using _idx_type =
    int32_t;
  _idx_type idx;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _x_type =
    int32_t;
  _x_type x;
  using _y_type =
    int32_t;
  _y_type y;
  using _width_type =
    int32_t;
  _width_type width;
  using _height_type =
    int32_t;
  _height_type height;
  using _armor_color_type =
    int32_t;
  _armor_color_type armor_color;
  using _car_x_type =
    int32_t;
  _car_x_type car_x;
  using _car_y_type =
    int32_t;
  _car_y_type car_y;
  using _car_width_type =
    int32_t;
  _car_width_type car_width;
  using _car_height_type =
    int32_t;
  _car_height_type car_height;
  using _world_x_type =
    float;
  _world_x_type world_x;
  using _world_y_type =
    float;
  _world_y_type world_y;
  using _fps_type =
    float;
  _fps_type fps;

  // setters for named parameter idiom
  Type & set__idx(
    const int32_t & _arg)
  {
    this->idx = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__x(
    const int32_t & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const int32_t & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__width(
    const int32_t & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__height(
    const int32_t & _arg)
  {
    this->height = _arg;
    return *this;
  }
  Type & set__armor_color(
    const int32_t & _arg)
  {
    this->armor_color = _arg;
    return *this;
  }
  Type & set__car_x(
    const int32_t & _arg)
  {
    this->car_x = _arg;
    return *this;
  }
  Type & set__car_y(
    const int32_t & _arg)
  {
    this->car_y = _arg;
    return *this;
  }
  Type & set__car_width(
    const int32_t & _arg)
  {
    this->car_width = _arg;
    return *this;
  }
  Type & set__car_height(
    const int32_t & _arg)
  {
    this->car_height = _arg;
    return *this;
  }
  Type & set__world_x(
    const float & _arg)
  {
    this->world_x = _arg;
    return *this;
  }
  Type & set__world_y(
    const float & _arg)
  {
    this->world_y = _arg;
    return *this;
  }
  Type & set__fps(
    const float & _arg)
  {
    this->fps = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator> *;
  using ConstRawPtr =
    const tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tensorrt_detect_msgs__msg__DetectionBox
    std::shared_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tensorrt_detect_msgs__msg__DetectionBox
    std::shared_ptr<tensorrt_detect_msgs::msg::DetectionBox_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DetectionBox_ & other) const
  {
    if (this->idx != other.idx) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->height != other.height) {
      return false;
    }
    if (this->armor_color != other.armor_color) {
      return false;
    }
    if (this->car_x != other.car_x) {
      return false;
    }
    if (this->car_y != other.car_y) {
      return false;
    }
    if (this->car_width != other.car_width) {
      return false;
    }
    if (this->car_height != other.car_height) {
      return false;
    }
    if (this->world_x != other.world_x) {
      return false;
    }
    if (this->world_y != other.world_y) {
      return false;
    }
    if (this->fps != other.fps) {
      return false;
    }
    return true;
  }
  bool operator!=(const DetectionBox_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DetectionBox_

// alias to use template instance with default allocator
using DetectionBox =
  tensorrt_detect_msgs::msg::DetectionBox_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__DETECTION_BOX__STRUCT_HPP_
