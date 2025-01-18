// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface_gym:msg/TrajMsg.idl
// generated code does not contain a copyright notice
#include "custom_interface_gym/msg/detail/traj_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
// Member `acceleration`
// Member `jerk`
// Member `snap`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
custom_interface_gym__msg__TrajMsg__init(custom_interface_gym__msg__TrajMsg * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->acceleration)) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
    return false;
  }
  // jerk
  if (!geometry_msgs__msg__Vector3__init(&msg->jerk)) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
    return false;
  }
  // snap
  if (!geometry_msgs__msg__Vector3__init(&msg->snap)) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
    return false;
  }
  // yaw
  // hover
  return true;
}

void
custom_interface_gym__msg__TrajMsg__fini(custom_interface_gym__msg__TrajMsg * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // acceleration
  geometry_msgs__msg__Vector3__fini(&msg->acceleration);
  // jerk
  geometry_msgs__msg__Vector3__fini(&msg->jerk);
  // snap
  geometry_msgs__msg__Vector3__fini(&msg->snap);
  // yaw
  // hover
}

bool
custom_interface_gym__msg__TrajMsg__are_equal(const custom_interface_gym__msg__TrajMsg * lhs, const custom_interface_gym__msg__TrajMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->acceleration), &(rhs->acceleration)))
  {
    return false;
  }
  // jerk
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->jerk), &(rhs->jerk)))
  {
    return false;
  }
  // snap
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->snap), &(rhs->snap)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // hover
  if (lhs->hover != rhs->hover) {
    return false;
  }
  return true;
}

bool
custom_interface_gym__msg__TrajMsg__copy(
  const custom_interface_gym__msg__TrajMsg * input,
  custom_interface_gym__msg__TrajMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->acceleration), &(output->acceleration)))
  {
    return false;
  }
  // jerk
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->jerk), &(output->jerk)))
  {
    return false;
  }
  // snap
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->snap), &(output->snap)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // hover
  output->hover = input->hover;
  return true;
}

custom_interface_gym__msg__TrajMsg *
custom_interface_gym__msg__TrajMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface_gym__msg__TrajMsg * msg = (custom_interface_gym__msg__TrajMsg *)allocator.allocate(sizeof(custom_interface_gym__msg__TrajMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface_gym__msg__TrajMsg));
  bool success = custom_interface_gym__msg__TrajMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface_gym__msg__TrajMsg__destroy(custom_interface_gym__msg__TrajMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface_gym__msg__TrajMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface_gym__msg__TrajMsg__Sequence__init(custom_interface_gym__msg__TrajMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface_gym__msg__TrajMsg * data = NULL;

  if (size) {
    data = (custom_interface_gym__msg__TrajMsg *)allocator.zero_allocate(size, sizeof(custom_interface_gym__msg__TrajMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface_gym__msg__TrajMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface_gym__msg__TrajMsg__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interface_gym__msg__TrajMsg__Sequence__fini(custom_interface_gym__msg__TrajMsg__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interface_gym__msg__TrajMsg__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interface_gym__msg__TrajMsg__Sequence *
custom_interface_gym__msg__TrajMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface_gym__msg__TrajMsg__Sequence * array = (custom_interface_gym__msg__TrajMsg__Sequence *)allocator.allocate(sizeof(custom_interface_gym__msg__TrajMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface_gym__msg__TrajMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface_gym__msg__TrajMsg__Sequence__destroy(custom_interface_gym__msg__TrajMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface_gym__msg__TrajMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface_gym__msg__TrajMsg__Sequence__are_equal(const custom_interface_gym__msg__TrajMsg__Sequence * lhs, const custom_interface_gym__msg__TrajMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface_gym__msg__TrajMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface_gym__msg__TrajMsg__Sequence__copy(
  const custom_interface_gym__msg__TrajMsg__Sequence * input,
  custom_interface_gym__msg__TrajMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface_gym__msg__TrajMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface_gym__msg__TrajMsg * data =
      (custom_interface_gym__msg__TrajMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface_gym__msg__TrajMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface_gym__msg__TrajMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface_gym__msg__TrajMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
