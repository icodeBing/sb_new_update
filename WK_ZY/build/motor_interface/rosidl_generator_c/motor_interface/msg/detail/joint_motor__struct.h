// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__STRUCT_H_
#define MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__STRUCT_H_

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
// Member 'joint_names'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/JointMotor in the package motor_interface.
typedef struct motor_interface__msg__JointMotor
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String joint_names[23];
  double kp[23];
  double kd[23];
  double position[23];
  double velocity[23];
  double effort[23];
} motor_interface__msg__JointMotor;

// Struct for a sequence of motor_interface__msg__JointMotor.
typedef struct motor_interface__msg__JointMotor__Sequence
{
  motor_interface__msg__JointMotor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} motor_interface__msg__JointMotor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__STRUCT_H_
