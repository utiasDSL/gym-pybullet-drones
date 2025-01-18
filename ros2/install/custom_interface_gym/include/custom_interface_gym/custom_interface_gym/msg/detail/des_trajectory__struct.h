// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__STRUCT_H_
#define CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ACTION_ADD'.
/**
  * Action command for the trajectory server.
  * Add a new trajectory
 */
enum
{
  custom_interface_gym__msg__DesTrajectory__ACTION_ADD = 1ul
};

/// Constant 'ACTION_ABORT'.
/**
  * Abort the current trajectory
 */
enum
{
  custom_interface_gym__msg__DesTrajectory__ACTION_ABORT = 2ul
};

/// Constant 'ACTION_WARN_START'.
/**
  * Warning: trajectory start issue
 */
enum
{
  custom_interface_gym__msg__DesTrajectory__ACTION_WARN_START = 3ul
};

/// Constant 'ACTION_WARN_FINAL'.
/**
  * Warning: trajectory final issue
 */
enum
{
  custom_interface_gym__msg__DesTrajectory__ACTION_WARN_FINAL = 4ul
};

/// Constant 'ACTION_WARN_IMPOSSIBLE'.
/**
  * Warning: impossible trajectory
 */
enum
{
  custom_interface_gym__msg__DesTrajectory__ACTION_WARN_IMPOSSIBLE = 5ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'matrices_flat'
// Member 'duration_vector'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'debug_info'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DesTrajectory in the package custom_interface_gym.
/**
  * Trajectory.msg
 */
typedef struct custom_interface_gym__msg__DesTrajectory
{
  std_msgs__msg__Header header;
  /// Trajectory ID, starts from "1".
  uint32_t trajectory_id;
  /// Action type
  uint32_t action;
  /// Trajectory order and segments.
  /// Polynomial order of the trajectory (D)
  uint32_t num_order;
  /// Number of trajectory segments
  uint32_t num_segment;
  /// Array of flattened matrices (as a single sequence of float64)
  /// Flattened sequence of matrix values
  rosidl_runtime_c__double__Sequence matrices_flat;
  /// Duration for each segment
  /// Duration for each segment
  rosidl_runtime_c__double__Sequence duration_vector;
  /// Debugging information
  /// Optional debug info (e.g., for warnings or issues)
  rosidl_runtime_c__String debug_info;
} custom_interface_gym__msg__DesTrajectory;

// Struct for a sequence of custom_interface_gym__msg__DesTrajectory.
typedef struct custom_interface_gym__msg__DesTrajectory__Sequence
{
  custom_interface_gym__msg__DesTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface_gym__msg__DesTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE_GYM__MSG__DETAIL__DES_TRAJECTORY__STRUCT_H_
