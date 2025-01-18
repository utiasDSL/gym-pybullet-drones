// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface_gym:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__BUILDER_HPP_
#define CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface_gym/srv/detail/uav_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface_gym
{

namespace srv
{

namespace builder
{

class Init_UavControl_Request_request
{
public:
  Init_UavControl_Request_request()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface_gym::srv::UavControl_Request request(::custom_interface_gym::srv::UavControl_Request::_request_type arg)
  {
    msg_.request = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface_gym::srv::UavControl_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface_gym::srv::UavControl_Request>()
{
  return custom_interface_gym::srv::builder::Init_UavControl_Request_request();
}

}  // namespace custom_interface_gym


namespace custom_interface_gym
{

namespace srv
{

namespace builder
{

class Init_UavControl_Response_response
{
public:
  Init_UavControl_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface_gym::srv::UavControl_Response response(::custom_interface_gym::srv::UavControl_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface_gym::srv::UavControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface_gym::srv::UavControl_Response>()
{
  return custom_interface_gym::srv::builder::Init_UavControl_Response_response();
}

}  // namespace custom_interface_gym

#endif  // CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__BUILDER_HPP_
