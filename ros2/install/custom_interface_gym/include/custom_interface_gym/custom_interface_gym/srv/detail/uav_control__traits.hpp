// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface_gym:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__TRAITS_HPP_
#define CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface_gym/srv/detail/uav_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interface_gym
{

namespace srv
{

inline void to_flow_style_yaml(
  const UavControl_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UavControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UavControl_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface_gym

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface_gym::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface_gym::srv::UavControl_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface_gym::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface_gym::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface_gym::srv::UavControl_Request & msg)
{
  return custom_interface_gym::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface_gym::srv::UavControl_Request>()
{
  return "custom_interface_gym::srv::UavControl_Request";
}

template<>
inline const char * name<custom_interface_gym::srv::UavControl_Request>()
{
  return "custom_interface_gym/srv/UavControl_Request";
}

template<>
struct has_fixed_size<custom_interface_gym::srv::UavControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interface_gym::srv::UavControl_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interface_gym::srv::UavControl_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interface_gym
{

namespace srv
{

inline void to_flow_style_yaml(
  const UavControl_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const UavControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const UavControl_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface_gym

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface_gym::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface_gym::srv::UavControl_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface_gym::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface_gym::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface_gym::srv::UavControl_Response & msg)
{
  return custom_interface_gym::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface_gym::srv::UavControl_Response>()
{
  return "custom_interface_gym::srv::UavControl_Response";
}

template<>
inline const char * name<custom_interface_gym::srv::UavControl_Response>()
{
  return "custom_interface_gym/srv/UavControl_Response";
}

template<>
struct has_fixed_size<custom_interface_gym::srv::UavControl_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interface_gym::srv::UavControl_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interface_gym::srv::UavControl_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interface_gym::srv::UavControl>()
{
  return "custom_interface_gym::srv::UavControl";
}

template<>
inline const char * name<custom_interface_gym::srv::UavControl>()
{
  return "custom_interface_gym/srv/UavControl";
}

template<>
struct has_fixed_size<custom_interface_gym::srv::UavControl>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interface_gym::srv::UavControl_Request>::value &&
    has_fixed_size<custom_interface_gym::srv::UavControl_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interface_gym::srv::UavControl>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interface_gym::srv::UavControl_Request>::value &&
    has_bounded_size<custom_interface_gym::srv::UavControl_Response>::value
  >
{
};

template<>
struct is_service<custom_interface_gym::srv::UavControl>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interface_gym::srv::UavControl_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interface_gym::srv::UavControl_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__TRAITS_HPP_
