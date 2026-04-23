// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tensorrt_detect_msgs:msg/WorldTargetArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "tensorrt_detect_msgs/msg/world_target_array.hpp"


#ifndef TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__STRUCT_HPP_
#define TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__STRUCT_HPP_

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
// Member 'targets'
#include "tensorrt_detect_msgs/msg/detail/world_target__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tensorrt_detect_msgs__msg__WorldTargetArray __attribute__((deprecated))
#else
# define DEPRECATED__tensorrt_detect_msgs__msg__WorldTargetArray __declspec(deprecated)
#endif

namespace tensorrt_detect_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldTargetArray_
{
  using Type = WorldTargetArray_<ContainerAllocator>;

  explicit WorldTargetArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit WorldTargetArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _targets_type =
    std::vector<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>>;
  _targets_type targets;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__targets(
    const std::vector<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<tensorrt_detect_msgs::msg::WorldTarget_<ContainerAllocator>>> & _arg)
  {
    this->targets = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tensorrt_detect_msgs__msg__WorldTargetArray
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tensorrt_detect_msgs__msg__WorldTargetArray
    std::shared_ptr<tensorrt_detect_msgs::msg::WorldTargetArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldTargetArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->targets != other.targets) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldTargetArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldTargetArray_

// alias to use template instance with default allocator
using WorldTargetArray =
  tensorrt_detect_msgs::msg::WorldTargetArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tensorrt_detect_msgs

#endif  // TENSORRT_DETECT_MSGS__MSG__DETAIL__WORLD_TARGET_ARRAY__STRUCT_HPP_
