#ifndef MOTORCOMM_H
#define MOTORCOMM_H

#include <rclcpp/rclcpp.hpp>
#include "motor_interface/msg/joint_motor.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <chrono>
#include "global/golbalParams.h"

using namespace std::chrono_literals;

using JointMotor = motor_interface::msg::JointMotor;

class MotorComm : public rclcpp::Node {
public:
    MotorComm(const std::string& node_name);
    ~MotorComm();

private:
    void init();

    void joint_motor_comm_callback(const JointMotor::SharedPtr msg);

    void timer_callback();

private:
    rclcpp::Subscription<JointMotor>::SharedPtr sub_;
    rclcpp::Publisher<JointMotor>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::mutex motor_mutex; // 仅MotorComm定时器使用，独立锁
};

#endif