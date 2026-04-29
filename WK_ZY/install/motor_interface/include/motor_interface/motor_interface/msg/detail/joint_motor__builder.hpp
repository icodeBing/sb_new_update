// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__BUILDER_HPP_
#define MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "motor_interface/msg/detail/joint_motor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace motor_interface
{

namespace msg
{

namespace builder
{

class Init_JointMotor_effort
{
public:
  explicit Init_JointMotor_effort(::motor_interface::msg::JointMotor & msg)
  : msg_(msg)
  {}
  ::motor_interface::msg::JointMotor effort(::motor_interface::msg::JointMotor::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return std::move(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

class Init_JointMotor_velocity
{
public:
  explicit Init_JointMotor_velocity(::motor_interface::msg::JointMotor & msg)
  : msg_(msg)
  {}
  Init_JointMotor_effort velocity(::motor_interface::msg::JointMotor::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_JointMotor_effort(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

class Init_JointMotor_position
{
public:
  explicit Init_JointMotor_position(::motor_interface::msg::JointMotor & msg)
  : msg_(msg)
  {}
  Init_JointMotor_velocity position(::motor_interface::msg::JointMotor::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_JointMotor_velocity(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

class Init_JointMotor_kd
{
public:
  explicit Init_JointMotor_kd(::motor_interface::msg::JointMotor & msg)
  : msg_(msg)
  {}
  Init_JointMotor_position kd(::motor_interface::msg::JointMotor::_kd_type arg)
  {
    msg_.kd = std::move(arg);
    return Init_JointMotor_position(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

class Init_JointMotor_kp
{
public:
  explicit Init_JointMotor_kp(::motor_interface::msg::JointMotor & msg)
  : msg_(msg)
  {}
  Init_JointMotor_kd kp(::motor_interface::msg::JointMotor::_kp_type arg)
  {
    msg_.kp = std::move(arg);
    return Init_JointMotor_kd(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

class Init_JointMotor_joint_names
{
public:
  explicit Init_JointMotor_joint_names(::motor_interface::msg::JointMotor & msg)
  : msg_(msg)
  {}
  Init_JointMotor_kp joint_names(::motor_interface::msg::JointMotor::_joint_names_type arg)
  {
    msg_.joint_names = std::move(arg);
    return Init_JointMotor_kp(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

class Init_JointMotor_header
{
public:
  Init_JointMotor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointMotor_joint_names header(::motor_interface::msg::JointMotor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_JointMotor_joint_names(msg_);
  }

private:
  ::motor_interface::msg::JointMotor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::motor_interface::msg::JointMotor>()
{
  return motor_interface::msg::builder::Init_JointMotor_header();
}

}  // namespace motor_interface

#endif  // MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__BUILDER_HPP_
