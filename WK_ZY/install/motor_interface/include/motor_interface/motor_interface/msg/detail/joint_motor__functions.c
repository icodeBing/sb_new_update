// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice
#include "motor_interface/msg/detail/joint_motor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"

bool
motor_interface__msg__JointMotor__init(motor_interface__msg__JointMotor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    motor_interface__msg__JointMotor__fini(msg);
    return false;
  }
  // joint_names
  for (size_t i = 0; i < 23; ++i) {
    if (!rosidl_runtime_c__String__init(&msg->joint_names[i])) {
      motor_interface__msg__JointMotor__fini(msg);
      return false;
    }
  }
  // kp
  // kd
  // position
  // velocity
  // effort
  return true;
}

void
motor_interface__msg__JointMotor__fini(motor_interface__msg__JointMotor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_names
  for (size_t i = 0; i < 23; ++i) {
    rosidl_runtime_c__String__fini(&msg->joint_names[i]);
  }
  // kp
  // kd
  // position
  // velocity
  // effort
}

bool
motor_interface__msg__JointMotor__are_equal(const motor_interface__msg__JointMotor * lhs, const motor_interface__msg__JointMotor * rhs)
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
  // joint_names
  for (size_t i = 0; i < 23; ++i) {
    if (!rosidl_runtime_c__String__are_equal(
        &(lhs->joint_names[i]), &(rhs->joint_names[i])))
    {
      return false;
    }
  }
  // kp
  for (size_t i = 0; i < 23; ++i) {
    if (lhs->kp[i] != rhs->kp[i]) {
      return false;
    }
  }
  // kd
  for (size_t i = 0; i < 23; ++i) {
    if (lhs->kd[i] != rhs->kd[i]) {
      return false;
    }
  }
  // position
  for (size_t i = 0; i < 23; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // velocity
  for (size_t i = 0; i < 23; ++i) {
    if (lhs->velocity[i] != rhs->velocity[i]) {
      return false;
    }
  }
  // effort
  for (size_t i = 0; i < 23; ++i) {
    if (lhs->effort[i] != rhs->effort[i]) {
      return false;
    }
  }
  return true;
}

bool
motor_interface__msg__JointMotor__copy(
  const motor_interface__msg__JointMotor * input,
  motor_interface__msg__JointMotor * output)
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
  // joint_names
  for (size_t i = 0; i < 23; ++i) {
    if (!rosidl_runtime_c__String__copy(
        &(input->joint_names[i]), &(output->joint_names[i])))
    {
      return false;
    }
  }
  // kp
  for (size_t i = 0; i < 23; ++i) {
    output->kp[i] = input->kp[i];
  }
  // kd
  for (size_t i = 0; i < 23; ++i) {
    output->kd[i] = input->kd[i];
  }
  // position
  for (size_t i = 0; i < 23; ++i) {
    output->position[i] = input->position[i];
  }
  // velocity
  for (size_t i = 0; i < 23; ++i) {
    output->velocity[i] = input->velocity[i];
  }
  // effort
  for (size_t i = 0; i < 23; ++i) {
    output->effort[i] = input->effort[i];
  }
  return true;
}

motor_interface__msg__JointMotor *
motor_interface__msg__JointMotor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_interface__msg__JointMotor * msg = (motor_interface__msg__JointMotor *)allocator.allocate(sizeof(motor_interface__msg__JointMotor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(motor_interface__msg__JointMotor));
  bool success = motor_interface__msg__JointMotor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
motor_interface__msg__JointMotor__destroy(motor_interface__msg__JointMotor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    motor_interface__msg__JointMotor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
motor_interface__msg__JointMotor__Sequence__init(motor_interface__msg__JointMotor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_interface__msg__JointMotor * data = NULL;

  if (size) {
    data = (motor_interface__msg__JointMotor *)allocator.zero_allocate(size, sizeof(motor_interface__msg__JointMotor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = motor_interface__msg__JointMotor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        motor_interface__msg__JointMotor__fini(&data[i - 1]);
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
motor_interface__msg__JointMotor__Sequence__fini(motor_interface__msg__JointMotor__Sequence * array)
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
      motor_interface__msg__JointMotor__fini(&array->data[i]);
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

motor_interface__msg__JointMotor__Sequence *
motor_interface__msg__JointMotor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  motor_interface__msg__JointMotor__Sequence * array = (motor_interface__msg__JointMotor__Sequence *)allocator.allocate(sizeof(motor_interface__msg__JointMotor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = motor_interface__msg__JointMotor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
motor_interface__msg__JointMotor__Sequence__destroy(motor_interface__msg__JointMotor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    motor_interface__msg__JointMotor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
motor_interface__msg__JointMotor__Sequence__are_equal(const motor_interface__msg__JointMotor__Sequence * lhs, const motor_interface__msg__JointMotor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!motor_interface__msg__JointMotor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
motor_interface__msg__JointMotor__Sequence__copy(
  const motor_interface__msg__JointMotor__Sequence * input,
  motor_interface__msg__JointMotor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(motor_interface__msg__JointMotor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    motor_interface__msg__JointMotor * data =
      (motor_interface__msg__JointMotor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!motor_interface__msg__JointMotor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          motor_interface__msg__JointMotor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!motor_interface__msg__JointMotor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
