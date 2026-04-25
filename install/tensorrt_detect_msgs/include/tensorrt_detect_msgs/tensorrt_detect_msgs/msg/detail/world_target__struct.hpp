// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tensorrt_detect_msgs:msg/WorldTarget.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__STRUCT_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tensorrt_detect_msgs__msg__WorldTarget __attribute__((deprecated))
#else
# define DEPRECATED__tensorrt_detect_msgs__msg__WorldTarget __declspec(deprecated)
#endif

namespace tensorrt_detect_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldTarget_
{
  using Type = WorldTarget_<ContainerAllocator>;

  explicit WorldTarget_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->idx = 0l;
      this->class_id = 0l;
      this->team_id = 0l;
      this->score = 0.0f;
      this->valid = false;
      this->world_x = 0.0f;
      this->world_y = 0.0f;
      this->world_z = 0.0f;
      this->bbox_x = 0l;
      this->bbox_y = 0l;
      this->bbox_w = 0l;
      this->bbox_h = 0l;
    }
  }

  explicit WorldTarget_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->idx = 0l;
      this->class_id = 0l;
      this->team_id = 0l;
      this->score = 0.0f;
      this->valid = false;
      this->world_x = 0.0f;
      this->world_y = 0.0f;
      this->world_z = 0.0f;
      this->bbox_x = 0l;
      this->bbox_y = 0l;
      this->bbox_w = 0l;
      this->bbox_h = 0l;
    }
  }

  // field types and members
  using _idx_type =
    int32_t;
  _idx_type idx;
  using _class_id_type =
    int32_t;
  _class_id_type class_id;
  using _team_id_type =
    int32_t;
  _team_id_type team_id;
  using _score_type =
    float;
  _score_type score;
  using _valid_type =
    bool;
  _valid_type valid;
  using _world_x_type =
    float;
  _world_x_type world_x;
  using _world_y_type =
    float;
  _world_y_type world_y;
  using _world_z_type =
    float;
  _world_z_type world_z;
  using _bbox_x_type =
    int32_t;
  _bbox_x_type bbox_x;
  using _bbox_y_type =
    int32_t;
  _bbox_y_type bbox_y;
  using _bbox_w_type =
    int32_t;
  _bbox_w_type bbox_w;
  using _bbox_h_type =
    int32_t;
  _bbox_h_type bbox_h;

  // setters for named parameter idiom
  Type & set__idx(
    const int32_t & _arg)
  {
    this->idx = _arg;
    return *this;
  }
  Type & set__class_id(
    const int32_t & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__team_id(
    const int32_t & _arg)
  {
    this->team_id = _arg;
    return *this;
  }
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__valid(
    const bool & _arg)
  {
    this->valid = _arg;
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
  Type & set__world_z(
    const float & _arg)
  {
    this->world_z = _arg;
    return *this;
  }
  Type & set__bbox_x(
    const int32_t & _arg)
  {
    this->bbox_x = _arg;
    return *this;
  }
  Type & set__bbox_y(
    const int32_t & _arg)
  {
    this->bbox_y = _arg;
    return *this;
  }
  Type & set__bbox_w(
    const int32_t & _arg)
  {
    this->bbox_w = _arg;
    return *this;
  }
  Type & set__bbox_h(
    const int32_t & _arg)
  {
    this->bbox_h = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator> *;
  using ConstRawPtr =
    const tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tensorrt_detect_msgs__msg__WorldTarget
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tensorrt_detect_msgs__msg__WorldTarget
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldTarget_ & other) const
  {
    if (this->idx != other.idx) {
      return false;
    }
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->team_id != other.team_id) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->valid != other.valid) {
      return false;
    }
    if (this->world_x != other.world_x) {
      return false;
    }
    if (this->world_y != other.world_y) {
      return false;
    }
    if (this->world_z != other.world_z) {
      return false;
    }
    if (this->bbox_x != other.bbox_x) {
      return false;
    }
    if (this->bbox_y != other.bbox_y) {
      return false;
    }
    if (this->bbox_w != other.bbox_w) {
      return false;
    }
    if (this->bbox_h != other.bbox_h) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldTarget_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldTarget_

// alias to use template instance with default allocator
using WorldTarget =
  tensorrt_detect_msgs::msg::WorldTarget_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET__STRUCT_HPP_
