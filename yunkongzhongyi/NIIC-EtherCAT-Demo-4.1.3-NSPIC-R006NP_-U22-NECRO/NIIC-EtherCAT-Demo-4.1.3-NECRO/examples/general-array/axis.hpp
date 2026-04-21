#pragma once

#include <ecat/types.hpp>
#include <map>

struct axis_data
{
  std::uint16_t axis_id;
  std::uint16_t slave_pos;

  // process data address
  volatile std::uint16_t* control_word = nullptr;
  volatile std::int32_t* target_position = nullptr;
  volatile std::int8_t* mode_of_operation = nullptr;
  volatile std::int32_t* target_velocity = nullptr;
  volatile std::int16_t* target_torque = nullptr;

  volatile const std::uint16_t* error_code = nullptr;
  volatile const std::uint16_t* status_word = nullptr;
  volatile const std::int32_t* position_actual_value = nullptr;
  volatile const std::int8_t* mode_of_operation_display = nullptr;
  volatile const std::int32_t* velocity_actual_value = nullptr;
  volatile const std::int16_t* torque_actual_value = nullptr;

  std::uint16_t old_status_word;
  int mode;
  bool power_enable;
  uint32_t freq = 0;
  int direction = 0;
  int motorBit = 17;
  bool quickStop = false;
};

enum mode_of_operation_type : std::uint8_t
{
  op_mode_no = 0,
  op_mode_pp = 1,
  op_mode_vl = 2,
  op_mode_pv = 3,
  op_mode_hm = 6,
  op_mode_ip = 7,
  op_mode_csp = 8,
  op_mode_csv = 9,
  op_mode_cst = 10,
};
