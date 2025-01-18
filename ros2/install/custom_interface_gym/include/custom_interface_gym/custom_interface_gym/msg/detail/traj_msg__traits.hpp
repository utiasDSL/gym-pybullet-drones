// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface_gym:msg/TrajMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__TRAITS_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface_gym/msg/detail/traj_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"
// Member 'velocity'
// Member 'acceleration'
// Member 'jerk'
// Member 'snap'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace custom_interface_gym
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrajMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: velocity
  {
    out << "velocity: ";
    to_flow_style_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: acceleration
  {
    out << "acceleration: ";
    to_flow_style_yaml(msg.acceleration, out);
    out << ", ";
  }

  // member: jerk
  {
    out << "jerk: ";
    to_flow_style_yaml(msg.jerk, out);
    out << ", ";
  }

  // member: snap
  {
    out << "snap: ";
    to_flow_style_yaml(msg.snap, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << ", ";
  }

  // member: hover
  {
    out << "hover: ";
    rosidl_generator_traits::value_to_yaml(msg.hover, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrajMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity:\n";
    to_block_style_yaml(msg.velocity, out, indentation + 2);
  }

  // member: acceleration
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "acceleration:\n";
    to_block_style_yaml(msg.acceleration, out, indentation + 2);
  }

  // member: jerk
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "jerk:\n";
    to_block_style_yaml(msg.jerk, out, indentation + 2);
  }

  // member: snap
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "snap:\n";
    to_block_style_yaml(msg.snap, out, indentation + 2);
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }

  // member: hover
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "hover: ";
    rosidl_generator_traits::value_to_yaml(msg.hover, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrajMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_interface_gym

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface_gym::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface_gym::msg::TrajMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface_gym::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface_gym::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface_gym::msg::TrajMsg & msg)
{
  return custom_interface_gym::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface_gym::msg::TrajMsg>()
{
  return "custom_interface_gym::msg::TrajMsg";
}

template<>
inline const char * name<custom_interface_gym::msg::TrajMsg>()
{
  return "custom_interface_gym/msg/TrajMsg";
}

template<>
struct has_fixed_size<custom_interface_gym::msg::TrajMsg>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<geometry_msgs::msg::Vector3>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<custom_interface_gym::msg::TrajMsg>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<geometry_msgs::msg::Vector3>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<custom_interface_gym::msg::TrajMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__TRAITS_HPP_
