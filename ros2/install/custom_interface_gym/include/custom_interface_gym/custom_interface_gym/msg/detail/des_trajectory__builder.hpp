// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__BUILDER_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface_gym/msg/detail/des_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface_gym
{

namespace msg
{

namespace builder
{

class Init_DesTrajectory_debug_info
{
public:
  explicit Init_DesTrajectory_debug_info(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  ::custom_interface_gym::msg::DesTrajectory debug_info(::custom_interface_gym::msg::DesTrajectory::_debug_info_type arg)
  {
    msg_.debug_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_duration_vector
{
public:
  explicit Init_DesTrajectory_duration_vector(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  Init_DesTrajectory_debug_info duration_vector(::custom_interface_gym::msg::DesTrajectory::_duration_vector_type arg)
  {
    msg_.duration_vector = std::move(arg);
    return Init_DesTrajectory_debug_info(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_matrices_flat
{
public:
  explicit Init_DesTrajectory_matrices_flat(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  Init_DesTrajectory_duration_vector matrices_flat(::custom_interface_gym::msg::DesTrajectory::_matrices_flat_type arg)
  {
    msg_.matrices_flat = std::move(arg);
    return Init_DesTrajectory_duration_vector(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_num_segment
{
public:
  explicit Init_DesTrajectory_num_segment(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  Init_DesTrajectory_matrices_flat num_segment(::custom_interface_gym::msg::DesTrajectory::_num_segment_type arg)
  {
    msg_.num_segment = std::move(arg);
    return Init_DesTrajectory_matrices_flat(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_num_order
{
public:
  explicit Init_DesTrajectory_num_order(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  Init_DesTrajectory_num_segment num_order(::custom_interface_gym::msg::DesTrajectory::_num_order_type arg)
  {
    msg_.num_order = std::move(arg);
    return Init_DesTrajectory_num_segment(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_action
{
public:
  explicit Init_DesTrajectory_action(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  Init_DesTrajectory_num_order action(::custom_interface_gym::msg::DesTrajectory::_action_type arg)
  {
    msg_.action = std::move(arg);
    return Init_DesTrajectory_num_order(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_trajectory_id
{
public:
  explicit Init_DesTrajectory_trajectory_id(::custom_interface_gym::msg::DesTrajectory & msg)
  : msg_(msg)
  {}
  Init_DesTrajectory_action trajectory_id(::custom_interface_gym::msg::DesTrajectory::_trajectory_id_type arg)
  {
    msg_.trajectory_id = std::move(arg);
    return Init_DesTrajectory_action(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

class Init_DesTrajectory_header
{
public:
  Init_DesTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DesTrajectory_trajectory_id header(::custom_interface_gym::msg::DesTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_DesTrajectory_trajectory_id(msg_);
  }

private:
  ::custom_interface_gym::msg::DesTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface_gym::msg::DesTrajectory>()
{
  return custom_interface_gym::msg::builder::Init_DesTrajectory_header();
}

}  // namespace custom_interface_gym

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__BUILDER_HPP_
