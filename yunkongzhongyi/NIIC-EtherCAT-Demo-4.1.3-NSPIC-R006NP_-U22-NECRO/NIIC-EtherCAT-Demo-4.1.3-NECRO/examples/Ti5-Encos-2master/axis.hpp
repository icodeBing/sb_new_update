#pragma once

#include <ecat/types.hpp>
#include <map>

struct axis_data
{
  std::uint16_t axis_id;
  std::uint16_t slave_pos;


  // process data address
  volatile std::uint16_t *control_word;
  volatile std::int32_t *target_position;

  volatile std::int16_t *target_torque;

  volatile std::int8_t *mode_of_operation;

  volatile const std::uint16_t *error_code;
  volatile const std::uint16_t *status_word;
  volatile const std::int32_t *position_actual_value;
  volatile const std::int8_t *mode_of_operation_display;

  volatile const std::int16_t *torque_actual_value;


 volatile const std::int32_t *velocity_actual_value;
   volatile std::int32_t *target_velocity;




std::uint16_t old_status_word;
                    int mode;
                    bool quickStop = false; 
                    bool power_enable = true;
};

#define encos_motor_num 12
struct encos_axis_data
{
  volatile double encos_angle_actual_value[encos_motor_num];
  volatile const std::int16_t *encos_torque_actual_value;
  volatile double encos_current_actual_value[encos_motor_num];
};
