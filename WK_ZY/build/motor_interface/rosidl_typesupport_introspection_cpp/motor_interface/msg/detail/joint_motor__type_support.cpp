// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "motor_interface/msg/detail/joint_motor__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace motor_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void JointMotor_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) motor_interface::msg::JointMotor(_init);
}

void JointMotor_fini_function(void * message_memory)
{
  auto typed_message = static_cast<motor_interface::msg::JointMotor *>(message_memory);
  typed_message->~JointMotor();
}

size_t size_function__JointMotor__joint_names(const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * get_const_function__JointMotor__joint_names(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<std::string, 23> *>(untyped_member);
  return &member[index];
}

void * get_function__JointMotor__joint_names(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<std::string, 23> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointMotor__joint_names(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const std::string *>(
    get_const_function__JointMotor__joint_names(untyped_member, index));
  auto & value = *reinterpret_cast<std::string *>(untyped_value);
  value = item;
}

void assign_function__JointMotor__joint_names(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<std::string *>(
    get_function__JointMotor__joint_names(untyped_member, index));
  const auto & value = *reinterpret_cast<const std::string *>(untyped_value);
  item = value;
}

size_t size_function__JointMotor__kp(const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * get_const_function__JointMotor__kp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void * get_function__JointMotor__kp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointMotor__kp(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointMotor__kp(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointMotor__kp(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointMotor__kp(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__JointMotor__kd(const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * get_const_function__JointMotor__kd(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void * get_function__JointMotor__kd(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointMotor__kd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointMotor__kd(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointMotor__kd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointMotor__kd(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__JointMotor__position(const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * get_const_function__JointMotor__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void * get_function__JointMotor__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointMotor__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointMotor__position(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointMotor__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointMotor__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__JointMotor__velocity(const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * get_const_function__JointMotor__velocity(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void * get_function__JointMotor__velocity(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointMotor__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointMotor__velocity(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointMotor__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointMotor__velocity(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__JointMotor__effort(const void * untyped_member)
{
  (void)untyped_member;
  return 23;
}

const void * get_const_function__JointMotor__effort(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void * get_function__JointMotor__effort(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 23> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointMotor__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointMotor__effort(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointMotor__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointMotor__effort(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JointMotor_message_member_array[7] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_names",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, joint_names),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointMotor__joint_names,  // size() function pointer
    get_const_function__JointMotor__joint_names,  // get_const(index) function pointer
    get_function__JointMotor__joint_names,  // get(index) function pointer
    fetch_function__JointMotor__joint_names,  // fetch(index, &value) function pointer
    assign_function__JointMotor__joint_names,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, kp),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointMotor__kp,  // size() function pointer
    get_const_function__JointMotor__kp,  // get_const(index) function pointer
    get_function__JointMotor__kp,  // get(index) function pointer
    fetch_function__JointMotor__kp,  // fetch(index, &value) function pointer
    assign_function__JointMotor__kp,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "kd",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, kd),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointMotor__kd,  // size() function pointer
    get_const_function__JointMotor__kd,  // get_const(index) function pointer
    get_function__JointMotor__kd,  // get(index) function pointer
    fetch_function__JointMotor__kd,  // fetch(index, &value) function pointer
    assign_function__JointMotor__kd,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointMotor__position,  // size() function pointer
    get_const_function__JointMotor__position,  // get_const(index) function pointer
    get_function__JointMotor__position,  // get(index) function pointer
    fetch_function__JointMotor__position,  // fetch(index, &value) function pointer
    assign_function__JointMotor__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, velocity),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointMotor__velocity,  // size() function pointer
    get_const_function__JointMotor__velocity,  // get_const(index) function pointer
    get_function__JointMotor__velocity,  // get(index) function pointer
    fetch_function__JointMotor__velocity,  // fetch(index, &value) function pointer
    assign_function__JointMotor__velocity,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "effort",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    23,  // array size
    false,  // is upper bound
    offsetof(motor_interface::msg::JointMotor, effort),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointMotor__effort,  // size() function pointer
    get_const_function__JointMotor__effort,  // get_const(index) function pointer
    get_function__JointMotor__effort,  // get(index) function pointer
    fetch_function__JointMotor__effort,  // fetch(index, &value) function pointer
    assign_function__JointMotor__effort,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JointMotor_message_members = {
  "motor_interface::msg",  // message namespace
  "JointMotor",  // message name
  7,  // number of fields
  sizeof(motor_interface::msg::JointMotor),
  JointMotor_message_member_array,  // message members
  JointMotor_init_function,  // function to initialize message memory (memory has to be allocated)
  JointMotor_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JointMotor_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JointMotor_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace motor_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<motor_interface::msg::JointMotor>()
{
  return &::motor_interface::msg::rosidl_typesupport_introspection_cpp::JointMotor_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, motor_interface, msg, JointMotor)() {
  return &::motor_interface::msg::rosidl_typesupport_introspection_cpp::JointMotor_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
