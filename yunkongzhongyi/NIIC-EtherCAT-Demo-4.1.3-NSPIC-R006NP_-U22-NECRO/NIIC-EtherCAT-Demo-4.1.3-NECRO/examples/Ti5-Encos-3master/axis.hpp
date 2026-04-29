#pragma once

#include <ecat/types.hpp>
#include <map>

enum Motor_Ctrl_Mode_type
{// 电机控制模式类型
    Pos_Mode,      // 位置控制模式
    Force_Pos_Mode // 力位混控模式
};

struct axis_data
{
  std::uint16_t axis_id;
  std::uint16_t slave_pos;
  std::uint16_t master_id;
  std::uint16_t old_status_word;

  // process data address
  volatile std::uint16_t *control_word;
  volatile std::int32_t *target_position;
  volatile std::int16_t *target_torque;
  volatile std::int32_t *target_velocity;

  volatile std::int8_t *mode_of_operation;

  volatile const std::uint16_t *error_code;
  volatile const std::uint16_t *status_word;
  volatile const std::int32_t *position_actual_value;
  volatile const std::int8_t *mode_of_operation_display;

  volatile const std::int16_t *torque_actual_value;
  volatile const std::int32_t *velocity_actual_value;

  // ==============================
  // 钛虎 C1 PT 力位混合模式 新增
  // 按照 钛虎C1关节电机通讯使用说明_0804.pdf
  // ==============================
  volatile float *kp;           // 0x200F:01 位置刚度  空载默认参考值
  volatile float *kd;           // 0x200F:02 阻尼系数  空载默认参考值
  volatile float *target_pos;      // 0x200F:03 目标位置（rad）
  volatile float *target_vel;      // 0x200F:04 目标速度（rad/s）
  volatile float *target_tor;      // 0x200F:05 目标力矩（N·m）

  volatile float *actual_pos;       // 0x2010:00 实际位置（rad）
  volatile float *actual_vel;       // 0x2011:00 实际速度（rad/s）
  volatile float *actual_cur;       // 0x2012:00 实际电流（A）

};

#define encos_motor_num 12
struct encos_axis_data
{
  volatile double encos_angle_actual_value[encos_motor_num];
  volatile double encos_angular_velocity_actual_value[encos_motor_num];
  volatile const std::int16_t *encos_torque_actual_value;
  volatile double encos_current_actual_value[encos_motor_num];
};
