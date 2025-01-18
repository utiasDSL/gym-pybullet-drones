// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface_gym:msg/TrajMsg.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__STRUCT_H_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
// Member 'acceleration'
// Member 'jerk'
// Member 'snap'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TrajMsg in the package custom_interface_gym.
/**
  * TrajMsg.msg
 */
typedef struct custom_interface_gym__msg__TrajMsg
{
  std_msgs__msg__Header header;
  /// Position (x, y, z)
  geometry_msgs__msg__Point position;
  /// Velocity (x, y, z)
  geometry_msgs__msg__Vector3 velocity;
  /// Acceleration (x, y, z)
  geometry_msgs__msg__Vector3 acceleration;
  /// Jerk (x, y, z)
  geometry_msgs__msg__Vector3 jerk;
  /// Snap (x, y, z)
  geometry_msgs__msg__Vector3 snap;
  /// Yaw
  float yaw;
  /// hover?
  bool hover;
} custom_interface_gym__msg__TrajMsg;

// Struct for a sequence of custom_interface_gym__msg__TrajMsg.
typedef struct custom_interface_gym__msg__TrajMsg__Sequence
{
  custom_interface_gym__msg__TrajMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_gym__msg__TrajMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__TRAJ_MSG__STRUCT_H_
