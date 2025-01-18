// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interface_gym:msg/DesTrajectory.idl
// generated code does not contain a copyright notice
#include "custom_interface_gym/msg/detail/des_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `matrices_flat`
// Member `duration_vector`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `debug_info`
#include "rosidl_runtime_c/string_functions.h"

bool
custom_interface_gym__msg__DesTrajectory__init(custom_interface_gym__msg__DesTrajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    custom_interface_gym__msg__DesTrajectory__fini(msg);
    return false;
  }
  // trajectory_id
  // action
  // num_order
  // num_segment
  // matrices_flat
  if (!rosidl_runtime_c__double__Sequence__init(&msg->matrices_flat, 0)) {
    custom_interface_gym__msg__DesTrajectory__fini(msg);
    return false;
  }
  // duration_vector
  if (!rosidl_runtime_c__double__Sequence__init(&msg->duration_vector, 0)) {
    custom_interface_gym__msg__DesTrajectory__fini(msg);
    return false;
  }
  // debug_info
  if (!rosidl_runtime_c__String__init(&msg->debug_info)) {
    custom_interface_gym__msg__DesTrajectory__fini(msg);
    return false;
  }
  return true;
}

void
custom_interface_gym__msg__DesTrajectory__fini(custom_interface_gym__msg__DesTrajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trajectory_id
  // action
  // num_order
  // num_segment
  // matrices_flat
  rosidl_runtime_c__double__Sequence__fini(&msg->matrices_flat);
  // duration_vector
  rosidl_runtime_c__double__Sequence__fini(&msg->duration_vector);
  // debug_info
  rosidl_runtime_c__String__fini(&msg->debug_info);
}

bool
custom_interface_gym__msg__DesTrajectory__are_equal(const custom_interface_gym__msg__DesTrajectory * lhs, const custom_interface_gym__msg__DesTrajectory * rhs)
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
  // trajectory_id
  if (lhs->trajectory_id != rhs->trajectory_id) {
    return false;
  }
  // action
  if (lhs->action != rhs->action) {
    return false;
  }
  // num_order
  if (lhs->num_order != rhs->num_order) {
    return false;
  }
  // num_segment
  if (lhs->num_segment != rhs->num_segment) {
    return false;
  }
  // matrices_flat
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->matrices_flat), &(rhs->matrices_flat)))
  {
    return false;
  }
  // duration_vector
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->duration_vector), &(rhs->duration_vector)))
  {
    return false;
  }
  // debug_info
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->debug_info), &(rhs->debug_info)))
  {
    return false;
  }
  return true;
}

bool
custom_interface_gym__msg__DesTrajectory__copy(
  const custom_interface_gym__msg__DesTrajectory * input,
  custom_interface_gym__msg__DesTrajectory * output)
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
  // trajectory_id
  output->trajectory_id = input->trajectory_id;
  // action
  output->action = input->action;
  // num_order
  output->num_order = input->num_order;
  // num_segment
  output->num_segment = input->num_segment;
  // matrices_flat
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->matrices_flat), &(output->matrices_flat)))
  {
    return false;
  }
  // duration_vector
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->duration_vector), &(output->duration_vector)))
  {
    return false;
  }
  // debug_info
  if (!rosidl_runtime_c__String__copy(
      &(input->debug_info), &(output->debug_info)))
  {
    return false;
  }
  return true;
}

custom_interface_gym__msg__DesTrajectory *
custom_interface_gym__msg__DesTrajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface_gym__msg__DesTrajectory * msg = (custom_interface_gym__msg__DesTrajectory *)allocator.allocate(sizeof(custom_interface_gym__msg__DesTrajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interface_gym__msg__DesTrajectory));
  bool success = custom_interface_gym__msg__DesTrajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interface_gym__msg__DesTrajectory__destroy(custom_interface_gym__msg__DesTrajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interface_gym__msg__DesTrajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interface_gym__msg__DesTrajectory__Sequence__init(custom_interface_gym__msg__DesTrajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface_gym__msg__DesTrajectory * data = NULL;

  if (size) {
    data = (custom_interface_gym__msg__DesTrajectory *)allocator.zero_allocate(size, sizeof(custom_interface_gym__msg__DesTrajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interface_gym__msg__DesTrajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interface_gym__msg__DesTrajectory__fini(&data[i - 1]);
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
custom_interface_gym__msg__DesTrajectory__Sequence__fini(custom_interface_gym__msg__DesTrajectory__Sequence * array)
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
      custom_interface_gym__msg__DesTrajectory__fini(&array->data[i]);
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

custom_interface_gym__msg__DesTrajectory__Sequence *
custom_interface_gym__msg__DesTrajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interface_gym__msg__DesTrajectory__Sequence * array = (custom_interface_gym__msg__DesTrajectory__Sequence *)allocator.allocate(sizeof(custom_interface_gym__msg__DesTrajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interface_gym__msg__DesTrajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interface_gym__msg__DesTrajectory__Sequence__destroy(custom_interface_gym__msg__DesTrajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interface_gym__msg__DesTrajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interface_gym__msg__DesTrajectory__Sequence__are_equal(const custom_interface_gym__msg__DesTrajectory__Sequence * lhs, const custom_interface_gym__msg__DesTrajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interface_gym__msg__DesTrajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interface_gym__msg__DesTrajectory__Sequence__copy(
  const custom_interface_gym__msg__DesTrajectory__Sequence * input,
  custom_interface_gym__msg__DesTrajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interface_gym__msg__DesTrajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interface_gym__msg__DesTrajectory * data =
      (custom_interface_gym__msg__DesTrajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interface_gym__msg__DesTrajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interface_gym__msg__DesTrajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interface_gym__msg__DesTrajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
