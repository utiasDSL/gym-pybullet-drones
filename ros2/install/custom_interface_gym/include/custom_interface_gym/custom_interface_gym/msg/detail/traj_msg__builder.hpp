// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface_gym:msg/TrajMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__BUILDER_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface_gym/msg/detail/traj_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface_gym
{

namespace msg
{

namespace builder
{

class Init_TrajMsg_hover
{
public:
  explicit Init_TrajMsg_hover(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  ::custom_interface_gym::msg::TrajMsg hover(::custom_interface_gym::msg::TrajMsg::_hover_type arg)
  {
    msg_.hover = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_yaw
{
public:
  explicit Init_TrajMsg_yaw(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  Init_TrajMsg_hover yaw(::custom_interface_gym::msg::TrajMsg::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_TrajMsg_hover(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_snap
{
public:
  explicit Init_TrajMsg_snap(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  Init_TrajMsg_yaw snap(::custom_interface_gym::msg::TrajMsg::_snap_type arg)
  {
    msg_.snap = std::move(arg);
    return Init_TrajMsg_yaw(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_jerk
{
public:
  explicit Init_TrajMsg_jerk(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  Init_TrajMsg_snap jerk(::custom_interface_gym::msg::TrajMsg::_jerk_type arg)
  {
    msg_.jerk = std::move(arg);
    return Init_TrajMsg_snap(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_acceleration
{
public:
  explicit Init_TrajMsg_acceleration(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  Init_TrajMsg_jerk acceleration(::custom_interface_gym::msg::TrajMsg::_acceleration_type arg)
  {
    msg_.acceleration = std::move(arg);
    return Init_TrajMsg_jerk(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_velocity
{
public:
  explicit Init_TrajMsg_velocity(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  Init_TrajMsg_acceleration velocity(::custom_interface_gym::msg::TrajMsg::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_TrajMsg_acceleration(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_position
{
public:
  explicit Init_TrajMsg_position(::custom_interface_gym::msg::TrajMsg & msg)
  : msg_(msg)
  {}
  Init_TrajMsg_velocity position(::custom_interface_gym::msg::TrajMsg::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_TrajMsg_velocity(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

class Init_TrajMsg_header
{
public:
  Init_TrajMsg_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrajMsg_position header(::custom_interface_gym::msg::TrajMsg::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrajMsg_position(msg_);
  }

private:
  ::custom_interface_gym::msg::TrajMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface_gym::msg::TrajMsg>()
{
  return custom_interface_gym::msg::builder::Init_TrajMsg_header();
}

}  // namespace custom_interface_gym

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__BUILDER_HPP_
