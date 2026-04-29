// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__FUNCTIONS_H_
#define MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "motor_interface/msg/rosidl_generator_c__visibility_control.h"

#include "motor_interface/msg/detail/joint_motor__struct.h"

/// Initialize msg/JointMotor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motor_interface__msg__JointMotor
 * )) before or use
 * motor_interface__msg__JointMotor__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
bool
motor_interface__msg__JointMotor__init(motor_interface__msg__JointMotor * msg);

/// Finalize msg/JointMotor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
void
motor_interface__msg__JointMotor__fini(motor_interface__msg__JointMotor * msg);

/// Create msg/JointMotor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motor_interface__msg__JointMotor__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
motor_interface__msg__JointMotor *
motor_interface__msg__JointMotor__create();

/// Destroy msg/JointMotor message.
/**
 * It calls
 * motor_interface__msg__JointMotor__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
void
motor_interface__msg__JointMotor__destroy(motor_interface__msg__JointMotor * msg);

/// Check for msg/JointMotor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
bool
motor_interface__msg__JointMotor__are_equal(const motor_interface__msg__JointMotor * lhs, const motor_interface__msg__JointMotor * rhs);

/// Copy a msg/JointMotor message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
bool
motor_interface__msg__JointMotor__copy(
  const motor_interface__msg__JointMotor * input,
  motor_interface__msg__JointMotor * output);

/// Initialize array of msg/JointMotor messages.
/**
 * It allocates the memory for the number of elements and calls
 * motor_interface__msg__JointMotor__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
bool
motor_interface__msg__JointMotor__Sequence__init(motor_interface__msg__JointMotor__Sequence * array, size_t size);

/// Finalize array of msg/JointMotor messages.
/**
 * It calls
 * motor_interface__msg__JointMotor__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
void
motor_interface__msg__JointMotor__Sequence__fini(motor_interface__msg__JointMotor__Sequence * array);

/// Create array of msg/JointMotor messages.
/**
 * It allocates the memory for the array and calls
 * motor_interface__msg__JointMotor__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
motor_interface__msg__JointMotor__Sequence *
motor_interface__msg__JointMotor__Sequence__create(size_t size);

/// Destroy array of msg/JointMotor messages.
/**
 * It calls
 * motor_interface__msg__JointMotor__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
void
motor_interface__msg__JointMotor__Sequence__destroy(motor_interface__msg__JointMotor__Sequence * array);

/// Check for msg/JointMotor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
bool
motor_interface__msg__JointMotor__Sequence__are_equal(const motor_interface__msg__JointMotor__Sequence * lhs, const motor_interface__msg__JointMotor__Sequence * rhs);

/// Copy an array of msg/JointMotor messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motor_interface
bool
motor_interface__msg__JointMotor__Sequence__copy(
  const motor_interface__msg__JointMotor__Sequence * input,
  motor_interface__msg__JointMotor__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__FUNCTIONS_H_
