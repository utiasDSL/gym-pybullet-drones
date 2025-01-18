// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface_gym:srv/UavControl.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__STRUCT_H_
#define CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/UavControl in the package custom_interface_gym.
typedef struct custom_interface_gym__srv__UavControl_Request
{
  /// Control inputs: [roll, pitch, yaw, throttle]
  float request[4];
} custom_interface_gym__srv__UavControl_Request;

// Struct for a sequence of custom_interface_gym__srv__UavControl_Request.
typedef struct custom_interface_gym__srv__UavControl_Request__Sequence
{
  custom_interface_gym__srv__UavControl_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_gym__srv__UavControl_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/UavControl in the package custom_interface_gym.
typedef struct custom_interface_gym__srv__UavControl_Response
{
  /// UAV state: [x, y, z, qx, qy, qz, qw]
  float response[7];
} custom_interface_gym__srv__UavControl_Response;

// Struct for a sequence of custom_interface_gym__srv__UavControl_Response.
typedef struct custom_interface_gym__srv__UavControl_Response__Sequence
{
  custom_interface_gym__srv__UavControl_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_gym__srv__UavControl_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE_GYM__SRV__DETAIL__UAV_CONTROL__STRUCT_H_
