// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interface_gym:srv/UavControl.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interface_gym/srv/detail/uav_control__rosidl_typesupport_introspection_c.h"
#include "custom_interface_gym/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interface_gym/srv/detail/uav_control__functions.h"
#include "custom_interface_gym/srv/detail/uav_control__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interface_gym__srv__UavControl_Request__init(message_memory);
}

void custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_fini_function(void * message_memory)
{
  custom_interface_gym__srv__UavControl_Request__fini(message_memory);
}

size_t custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__size_function__UavControl_Request__request(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__get_const_function__UavControl_Request__request(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__get_function__UavControl_Request__request(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__fetch_function__UavControl_Request__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__get_const_function__UavControl_Request__request(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__assign_function__UavControl_Request__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__get_function__UavControl_Request__request(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_member_array[1] = {
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__srv__UavControl_Request, request),  // bytes offset in struct
    NULL,  // default value
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__size_function__UavControl_Request__request,  // size() function pointer
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__get_const_function__UavControl_Request__request,  // get_const(index) function pointer
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__get_function__UavControl_Request__request,  // get(index) function pointer
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__fetch_function__UavControl_Request__request,  // fetch(index, &value) function pointer
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__assign_function__UavControl_Request__request,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_members = {
  "custom_interface_gym__srv",  // message namespace
  "UavControl_Request",  // message name
  1,  // number of fields
  sizeof(custom_interface_gym__srv__UavControl_Request),
  custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_member_array,  // message members
  custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_type_support_handle = {
  0,
  &custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface_gym
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl_Request)() {
  if (!custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_type_support_handle.typesupport_identifier) {
    custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interface_gym__srv__UavControl_Request__rosidl_typesupport_introspection_c__UavControl_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interface_gym/srv/detail/uav_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interface_gym/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interface_gym/srv/detail/uav_control__functions.h"
// already included above
// #include "custom_interface_gym/srv/detail/uav_control__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interface_gym__srv__UavControl_Response__init(message_memory);
}

void custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_fini_function(void * message_memory)
{
  custom_interface_gym__srv__UavControl_Response__fini(message_memory);
}

size_t custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__size_function__UavControl_Response__response(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__get_const_function__UavControl_Response__response(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__get_function__UavControl_Response__response(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__fetch_function__UavControl_Response__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__get_const_function__UavControl_Response__response(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__assign_function__UavControl_Response__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__get_function__UavControl_Response__response(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_member_array[1] = {
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(custom_interface_gym__srv__UavControl_Response, response),  // bytes offset in struct
    NULL,  // default value
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__size_function__UavControl_Response__response,  // size() function pointer
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__get_const_function__UavControl_Response__response,  // get_const(index) function pointer
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__get_function__UavControl_Response__response,  // get(index) function pointer
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__fetch_function__UavControl_Response__response,  // fetch(index, &value) function pointer
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__assign_function__UavControl_Response__response,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_members = {
  "custom_interface_gym__srv",  // message namespace
  "UavControl_Response",  // message name
  1,  // number of fields
  sizeof(custom_interface_gym__srv__UavControl_Response),
  custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_member_array,  // message members
  custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_type_support_handle = {
  0,
  &custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface_gym
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl_Response)() {
  if (!custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_type_support_handle.typesupport_identifier) {
    custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interface_gym__srv__UavControl_Response__rosidl_typesupport_introspection_c__UavControl_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interface_gym/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interface_gym/srv/detail/uav_control__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_members = {
  "custom_interface_gym__srv",  // service namespace
  "UavControl",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_Request_message_type_support_handle,
  NULL  // response message
  // custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_type_support_handle = {
  0,
  &custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interface_gym
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl)() {
  if (!custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_type_support_handle.typesupport_identifier) {
    custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interface_gym, srv, UavControl_Response)()->data;
  }

  return &custom_interface_gym__srv__detail__uav_control__rosidl_typesupport_introspection_c__UavControl_service_type_support_handle;
}
