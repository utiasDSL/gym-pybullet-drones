// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface_gym:msg/TrajMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__STRUCT_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"
// Member 'velocity'
// Member 'acceleration'
// Member 'jerk'
// Member 'snap'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__custom_interface_gym__msg__TrajMsg __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface_gym__msg__TrajMsg __declspec(deprecated)
#endif

namespace custom_interface_gym
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrajMsg_
{
  using Type = TrajMsg_<ContainerAllocator>;

  explicit TrajMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init),
    velocity(_init),
    acceleration(_init),
    jerk(_init),
    snap(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0f;
      this->hover = false;
    }
  }

  explicit TrajMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    velocity(_alloc, _init),
    acceleration(_alloc, _init),
    jerk(_alloc, _init),
    snap(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->yaw = 0.0f;
      this->hover = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _velocity_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _velocity_type velocity;
  using _acceleration_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _acceleration_type acceleration;
  using _jerk_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _jerk_type jerk;
  using _snap_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _snap_type snap;
  using _yaw_type =
    float;
  _yaw_type yaw;
  using _hover_type =
    bool;
  _hover_type hover;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__acceleration(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__jerk(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->jerk = _arg;
    return *this;
  }
  Type & set__snap(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->snap = _arg;
    return *this;
  }
  Type & set__yaw(
    const float & _arg)
  {
    this->yaw = _arg;
    return *this;
  }
  Type & set__hover(
    const bool & _arg)
  {
    this->hover = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface_gym::msg::TrajMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface_gym::msg::TrajMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::msg::TrajMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::msg::TrajMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface_gym__msg__TrajMsg
    std::shared_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface_gym__msg__TrajMsg
    std::shared_ptr<custom_interface_gym::msg::TrajMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrajMsg_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->jerk != other.jerk) {
      return false;
    }
    if (this->snap != other.snap) {
      return false;
    }
    if (this->yaw != other.yaw) {
      return false;
    }
    if (this->hover != other.hover) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrajMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrajMsg_

// alias to use template instance with default allocator
using TrajMsg =
  custom_interface_gym::msg::TrajMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interface_gym

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__STRUCT_HPP_
