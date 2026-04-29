// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "motor_interface/msg/detail/joint_motor__rosidl_typesupport_introspection_c.h"
#include "motor_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "motor_interface/msg/detail/joint_motor__functions.h"
#include "motor_interface/msg/detail/joint_motor__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `joint_names`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  motor_interface__msg__JointMotor__init(message_memory);
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_fini_function(void * message_memory)
{
  motor_interface__msg__JointMotor__fini(message_memory);
}

size_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__joint_names(
  const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__joint_names(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__String * member =
    (const rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__joint_names(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__String * member =
    (rosidl_runtime_c__String *)(untyped_member);
  return &member[index];
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__joint_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const rosidl_runtime_c__String * item =
    ((const rosidl_runtime_c__String *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__joint_names(untyped_member, index));
  rosidl_runtime_c__String * value =
    (rosidl_runtime_c__String *)(untyped_value);
  *value = *item;
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__joint_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  rosidl_runtime_c__String * item =
    ((rosidl_runtime_c__String *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__joint_names(untyped_member, index));
  const rosidl_runtime_c__String * value =
    (const rosidl_runtime_c__String *)(untyped_value);
  *item = *value;
}

size_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__kp(
  const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__kp(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__kp(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__kp(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__kp(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__kd(
  const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__kd(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__kd(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__kd(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__kd(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__position(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__position(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__velocity(
  const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__velocity(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__velocity(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__effort(
  const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__effort(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__effort(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_names",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, joint_names),  // bytes offset in struct
    NULL,  // default value
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__joint_names,  // size() function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__joint_names,  // get_const(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__joint_names,  // get(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__joint_names,  // fetch(index, &value) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__joint_names,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, kp),  // bytes offset in struct
    NULL,  // default value
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__kp,  // size() function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__kp,  // get_const(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__kp,  // get(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__kp,  // fetch(index, &value) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__kp,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "kd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, kd),  // bytes offset in struct
    NULL,  // default value
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__kd,  // size() function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__kd,  // get_const(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__kd,  // get(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__kd,  // fetch(index, &value) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__kd,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, position),  // bytes offset in struct
    NULL,  // default value
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__position,  // size() function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__position,  // get_const(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__position,  // get(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__position,  // fetch(index, &value) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, velocity),  // bytes offset in struct
    NULL,  // default value
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__velocity,  // size() function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__velocity,  // get_const(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__velocity,  // get(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__velocity,  // fetch(index, &value) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__velocity,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface__msg__JointMotor, effort),  // bytes offset in struct
    NULL,  // default value
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__size_function__JointMotor__effort,  // size() function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_const_function__JointMotor__effort,  // get_const(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__get_function__JointMotor__effort,  // get(index) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__fetch_function__JointMotor__effort,  // fetch(index, &value) function pointer
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__assign_function__JointMotor__effort,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_members = {
  "motor_interface__msg",  // message namespace
  "JointMotor",  // message name
  7,  // number of fields
  sizeof(motor_interface__msg__JointMotor),
  motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_member_array,  // message members
  motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_init_function,  // function to initialize message memory (memory has to be allocated)
  motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_type_support_handle = {
  0,
  &motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_motor_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, motor_interface, msg, JointMotor)() {
  motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_type_support_handle.typesupport_identifier) {
    motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &motor_interface__msg__JointMotor__rosidl_typesupport_introspection_c__JointMotor_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
