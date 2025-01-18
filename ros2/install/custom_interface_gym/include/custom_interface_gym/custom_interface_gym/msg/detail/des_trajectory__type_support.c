// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interface_gym/msg/detail/des_trajectory__rosidl_typesupport_introspection_c.h"
#include "custom_interface_gym/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interface_gym/msg/detail/des_trajectory__functions.h"
#include "custom_interface_gym/msg/detail/des_trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `matrices_flat`
// Member `duration_vector`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `debug_info`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interface_gym__msg__DesTrajectory__init(message_memory);
}

void custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_fini_function(void * message_memory)
{
  custom_interface_gym__msg__DesTrajectory__fini(message_memory);
}

size_t custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__size_function__DesTrajectory__matrices_flat(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_const_function__DesTrajectory__matrices_flat(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_function__DesTrajectory__matrices_flat(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__fetch_function__DesTrajectory__matrices_flat(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_const_function__DesTrajectory__matrices_flat(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__assign_function__DesTrajectory__matrices_flat(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_function__DesTrajectory__matrices_flat(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__resize_function__DesTrajectory__matrices_flat(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__size_function__DesTrajectory__duration_vector(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_const_function__DesTrajectory__duration_vector(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_function__DesTrajectory__duration_vector(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__fetch_function__DesTrajectory__duration_vector(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_const_function__DesTrajectory__duration_vector(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__assign_function__DesTrajectory__duration_vector(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_function__DesTrajectory__duration_vector(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__resize_function__DesTrajectory__duration_vector(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trajectory_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, trajectory_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_order",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, num_order),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_segment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, num_segment),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "matrices_flat",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, matrices_flat),  // bytes offset in struct
    NULL,  // default value
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__size_function__DesTrajectory__matrices_flat,  // size() function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_const_function__DesTrajectory__matrices_flat,  // get_const(index) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_function__DesTrajectory__matrices_flat,  // get(index) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__fetch_function__DesTrajectory__matrices_flat,  // fetch(index, &value) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__assign_function__DesTrajectory__matrices_flat,  // assign(index, value) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__resize_function__DesTrajectory__matrices_flat  // resize(index) function pointer
  },
  {
    "duration_vector",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, duration_vector),  // bytes offset in struct
    NULL,  // default value
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__size_function__DesTrajectory__duration_vector,  // size() function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_const_function__DesTrajectory__duration_vector,  // get_const(index) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__get_function__DesTrajectory__duration_vector,  // get(index) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__fetch_function__DesTrajectory__duration_vector,  // fetch(index, &value) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__assign_function__DesTrajectory__duration_vector,  // assign(index, value) function pointer
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__resize_function__DesTrajectory__duration_vector  // resize(index) function pointer
  },
  {
    "debug_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__msg__DesTrajectory, debug_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_members = {
  "custom_interface_gym__msg",  // message namespace
  "DesTrajectory",  // message name
  8,  // number of fields
  sizeof(custom_interface_gym__msg__DesTrajectory),
  custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_member_array,  // message members
  custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_type_support_handle = {
  0,
  &custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface_gym
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, msg, DesTrajectory)() {
  custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_type_support_handle.typesupport_identifier) {
    custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interface_gym__msg__DesTrajectory__rosidl_typesupport_introspection_c__DesTrajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
