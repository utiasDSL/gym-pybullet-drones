// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "custom_interface_gym/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "custom_interface_gym/msg/detail/des_trajectory__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace custom_interface_gym
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_interface_gym
cdr_serialize(
  const custom_interface_gym::msg::DesTrajectory & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_interface_gym
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  custom_interface_gym::msg::DesTrajectory & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_interface_gym
get_serialized_size(
  const custom_interface_gym::msg::DesTrajectory & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_interface_gym
max_serialized_size_DesTrajectory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace custom_interface_gym

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_custom_interface_gym
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, custom_interface_gym, msg, DesTrajectory)();

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
