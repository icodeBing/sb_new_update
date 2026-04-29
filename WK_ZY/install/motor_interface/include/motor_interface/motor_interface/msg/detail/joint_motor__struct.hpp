// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from motor_interface:msg/JointMotor.idl
// generated code does not contain a copyright notice

#ifndef MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__STRUCT_HPP_
#define MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__motor_interface__msg__JointMotor __attribute__((deprecated))
#else
# define DEPRECATED__motor_interface__msg__JointMotor __declspec(deprecated)
#endif

namespace motor_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointMotor_
{
  using Type = JointMotor_<ContainerAllocator>;

  explicit JointMotor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 23>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->joint_names.begin(), this->joint_names.end(), "");
      std::fill<typename std::array<double, 23>::iterator, double>(this->kp.begin(), this->kp.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->kd.begin(), this->kd.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->position.begin(), this->position.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->velocity.begin(), this->velocity.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->effort.begin(), this->effort.end(), 0.0);
    }
  }

  explicit JointMotor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    joint_names(_alloc),
    kp(_alloc),
    kd(_alloc),
    position(_alloc),
    velocity(_alloc),
    effort(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 23>::iterator, std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>(this->joint_names.begin(), this->joint_names.end(), "");
      std::fill<typename std::array<double, 23>::iterator, double>(this->kp.begin(), this->kp.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->kd.begin(), this->kd.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->position.begin(), this->position.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->velocity.begin(), this->velocity.end(), 0.0);
      std::fill<typename std::array<double, 23>::iterator, double>(this->effort.begin(), this->effort.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_names_type =
    std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 23>;
  _joint_names_type joint_names;
  using _kp_type =
    std::array<double, 23>;
  _kp_type kp;
  using _kd_type =
    std::array<double, 23>;
  _kd_type kd;
  using _position_type =
    std::array<double, 23>;
  _position_type position;
  using _velocity_type =
    std::array<double, 23>;
  _velocity_type velocity;
  using _effort_type =
    std::array<double, 23>;
  _effort_type effort;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_names(
    const std::array<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, 23> & _arg)
  {
    this->joint_names = _arg;
    return *this;
  }
  Type & set__kp(
    const std::array<double, 23> & _arg)
  {
    this->kp = _arg;
    return *this;
  }
  Type & set__kd(
    const std::array<double, 23> & _arg)
  {
    this->kd = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<double, 23> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::array<double, 23> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const std::array<double, 23> & _arg)
  {
    this->effort = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    motor_interface::msg::JointMotor_<ContainerAllocator> *;
  using ConstRawPtr =
    const motor_interface::msg::JointMotor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<motor_interface::msg::JointMotor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<motor_interface::msg::JointMotor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      motor_interface::msg::JointMotor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<motor_interface::msg::JointMotor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      motor_interface::msg::JointMotor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<motor_interface::msg::JointMotor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<motor_interface::msg::JointMotor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<motor_interface::msg::JointMotor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__motor_interface__msg__JointMotor
    std::shared_ptr<motor_interface::msg::JointMotor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__motor_interface__msg__JointMotor
    std::shared_ptr<motor_interface::msg::JointMotor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointMotor_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_names != other.joint_names) {
      return false;
    }
    if (this->kp != other.kp) {
      return false;
    }
    if (this->kd != other.kd) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointMotor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointMotor_

// alias to use template instance with default allocator
using JointMotor =
  motor_interface::msg::JointMotor_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace motor_interface

#endif  // MOTOR_INTERFACE__MSG__DETAIL__JOINT_MOTOR__STRUCT_HPP_
