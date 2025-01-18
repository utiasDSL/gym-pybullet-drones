// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__TRAITS_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface_gym/msg/detail/des_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace custom_interface_gym
{

namespace msg
{

inline void to_flow_style_yaml(
  const DesTrajectory & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: trajectory_id
  {
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << ", ";
  }

  // member: action
  {
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << ", ";
  }

  // member: num_order
  {
    out << "num_order: ";
    rosidl_generator_traits::value_to_yaml(msg.num_order, out);
    out << ", ";
  }

  // member: num_segment
  {
    out << "num_segment: ";
    rosidl_generator_traits::value_to_yaml(msg.num_segment, out);
    out << ", ";
  }

  // member: matrices_flat
  {
    if (msg.matrices_flat.size() == 0) {
      out << "matrices_flat: []";
    } else {
      out << "matrices_flat: [";
      size_t pending_items = msg.matrices_flat.size();
      for (auto item : msg.matrices_flat) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: duration_vector
  {
    if (msg.duration_vector.size() == 0) {
      out << "duration_vector: []";
    } else {
      out << "duration_vector: [";
      size_t pending_items = msg.duration_vector.size();
      for (auto item : msg.duration_vector) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: debug_info
  {
    out << "debug_info: ";
    rosidl_generator_traits::value_to_yaml(msg.debug_info, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DesTrajectory & msg,
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

  // member: trajectory_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_id: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_id, out);
    out << "\n";
  }

  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "action: ";
    rosidl_generator_traits::value_to_yaml(msg.action, out);
    out << "\n";
  }

  // member: num_order
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_order: ";
    rosidl_generator_traits::value_to_yaml(msg.num_order, out);
    out << "\n";
  }

  // member: num_segment
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_segment: ";
    rosidl_generator_traits::value_to_yaml(msg.num_segment, out);
    out << "\n";
  }

  // member: matrices_flat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.matrices_flat.size() == 0) {
      out << "matrices_flat: []\n";
    } else {
      out << "matrices_flat:\n";
      for (auto item : msg.matrices_flat) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: duration_vector
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.duration_vector.size() == 0) {
      out << "duration_vector: []\n";
    } else {
      out << "duration_vector:\n";
      for (auto item : msg.duration_vector) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: debug_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "debug_info: ";
    rosidl_generator_traits::value_to_yaml(msg.debug_info, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DesTrajectory & msg, bool use_flow_style = false)
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
  const custom_interface_gym::msg::DesTrajectory & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface_gym::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface_gym::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface_gym::msg::DesTrajectory & msg)
{
  return custom_interface_gym::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface_gym::msg::DesTrajectory>()
{
  return "custom_interface_gym::msg::DesTrajectory";
}

template<>
inline const char * name<custom_interface_gym::msg::DesTrajectory>()
{
  return "custom_interface_gym/msg/DesTrajectory";
}

template<>
struct has_fixed_size<custom_interface_gym::msg::DesTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface_gym::msg::DesTrajectory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface_gym::msg::DesTrajectory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__TRAITS_HPP_
