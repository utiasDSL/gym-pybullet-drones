// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interface_gym:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__STRUCT_HPP_
#define CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interface_gym__srv__UavControl_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface_gym__srv__UavControl_Request __declspec(deprecated)
#endif

namespace custom_interface_gym
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UavControl_Request_
{
  using Type = UavControl_Request_<ContainerAllocator>;

  explicit UavControl_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->request.begin(), this->request.end(), 0.0f);
    }
  }

  explicit UavControl_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : request(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 4>::iterator, float>(this->request.begin(), this->request.end(), 0.0f);
    }
  }

  // field types and members
  using _request_type =
    std::array<float, 4>;
  _request_type request;

  // setters for named parameter idiom
  Type & set__request(
    const std::array<float, 4> & _arg)
  {
    this->request = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface_gym::srv::UavControl_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface_gym::srv::UavControl_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::srv::UavControl_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::srv::UavControl_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface_gym__srv__UavControl_Request
    std::shared_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface_gym__srv__UavControl_Request
    std::shared_ptr<custom_interface_gym::srv::UavControl_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UavControl_Request_ & other) const
  {
    if (this->request != other.request) {
      return false;
    }
    return true;
  }
  bool operator!=(const UavControl_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UavControl_Request_

// alias to use template instance with default allocator
using UavControl_Request =
  custom_interface_gym::srv::UavControl_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interface_gym


#ifndef _WIN32
# define DEPRECATED__custom_interface_gym__srv__UavControl_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_interface_gym__srv__UavControl_Response __declspec(deprecated)
#endif

namespace custom_interface_gym
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct UavControl_Response_
{
  using Type = UavControl_Response_<ContainerAllocator>;

  explicit UavControl_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->response.begin(), this->response.end(), 0.0f);
    }
  }

  explicit UavControl_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : response(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 7>::iterator, float>(this->response.begin(), this->response.end(), 0.0f);
    }
  }

  // field types and members
  using _response_type =
    std::array<float, 7>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__response(
    const std::array<float, 7> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interface_gym::srv::UavControl_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interface_gym::srv::UavControl_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::srv::UavControl_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interface_gym::srv::UavControl_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interface_gym__srv__UavControl_Response
    std::shared_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interface_gym__srv__UavControl_Response
    std::shared_ptr<custom_interface_gym::srv::UavControl_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const UavControl_Response_ & other) const
  {
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const UavControl_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct UavControl_Response_

// alias to use template instance with default allocator
using UavControl_Response =
  custom_interface_gym::srv::UavControl_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_interface_gym

namespace custom_interface_gym
{

namespace srv
{

struct UavControl
{
  using Request = custom_interface_gym::srv::UavControl_Request;
  using Response = custom_interface_gym::srv::UavControl_Response;
};

}  // namespace srv

}  // namespace custom_interface_gym

#endif  // CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__STRUCT_HPP_
