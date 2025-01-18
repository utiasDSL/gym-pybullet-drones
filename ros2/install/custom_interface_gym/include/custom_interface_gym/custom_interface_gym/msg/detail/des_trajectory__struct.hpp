// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__STRUCT_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__STRUCT_HPP_

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

#ifndef _WIN32
# define DEPRECATED__custom_interface_gym__msg__DesTrajectory __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface_gym__msg__DesTrajectory __declspec(deprecated)
#endif

namespace custom_interface_gym
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DesTrajectory_
{
  using Type = DesTrajectory_<ContainerAllocator>;

  explicit DesTrajectory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0ul;
      this->action = 0ul;
      this->num_order = 0ul;
      this->num_segment = 0ul;
      this->debug_info = "";
    }
  }

  explicit DesTrajectory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    debug_info(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->trajectory_id = 0ul;
      this->action = 0ul;
      this->num_order = 0ul;
      this->num_segment = 0ul;
      this->debug_info = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trajectory_id_type =
    uint32_t;
  _trajectory_id_type trajectory_id;
  using _action_type =
    uint32_t;
  _action_type action;
  using _num_order_type =
    uint32_t;
  _num_order_type num_order;
  using _num_segment_type =
    uint32_t;
  _num_segment_type num_segment;
  using _matrices_flat_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _matrices_flat_type matrices_flat;
  using _duration_vector_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _duration_vector_type duration_vector;
  using _debug_info_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _debug_info_type debug_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trajectory_id(
    const uint32_t & _arg)
  {
    this->trajectory_id = _arg;
    return *this;
  }
  Type & set__action(
    const uint32_t & _arg)
  {
    this->action = _arg;
    return *this;
  }
  Type & set__num_order(
    const uint32_t & _arg)
  {
    this->num_order = _arg;
    return *this;
  }
  Type & set__num_segment(
    const uint32_t & _arg)
  {
    this->num_segment = _arg;
    return *this;
  }
  Type & set__matrices_flat(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->matrices_flat = _arg;
    return *this;
  }
  Type & set__duration_vector(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->duration_vector = _arg;
    return *this;
  }
  Type & set__debug_info(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->debug_info = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint32_t ACTION_ADD =
    1u;
  static constexpr uint32_t ACTION_ABORT =
    2u;
  static constexpr uint32_t ACTION_WARN_START =
    3u;
  static constexpr uint32_t ACTION_WARN_FINAL =
    4u;
  static constexpr uint32_t ACTION_WARN_IMPOSSIBLE =
    5u;

  // pointer types
  using RawPtr =
    custom_interface_gym::msg::DesTrajectory_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface_gym::msg::DesTrajectory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::msg::DesTrajectory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::msg::DesTrajectory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface_gym__msg__DesTrajectory
    std::shared_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface_gym__msg__DesTrajectory
    std::shared_ptr<custom_interface_gym::msg::DesTrajectory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DesTrajectory_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trajectory_id != other.trajectory_id) {
      return false;
    }
    if (this->action != other.action) {
      return false;
    }
    if (this->num_order != other.num_order) {
      return false;
    }
    if (this->num_segment != other.num_segment) {
      return false;
    }
    if (this->matrices_flat != other.matrices_flat) {
      return false;
    }
    if (this->duration_vector != other.duration_vector) {
      return false;
    }
    if (this->debug_info != other.debug_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const DesTrajectory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DesTrajectory_

// alias to use template instance with default allocator
using DesTrajectory =
  custom_interface_gym::msg::DesTrajectory_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t DesTrajectory_<ContainerAllocator>::ACTION_ADD;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t DesTrajectory_<ContainerAllocator>::ACTION_ABORT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t DesTrajectory_<ContainerAllocator>::ACTION_WARN_START;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t DesTrajectory_<ContainerAllocator>::ACTION_WARN_FINAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint32_t DesTrajectory_<ContainerAllocator>::ACTION_WARN_IMPOSSIBLE;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace custom_interface_gym

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__STRUCT_HPP_
