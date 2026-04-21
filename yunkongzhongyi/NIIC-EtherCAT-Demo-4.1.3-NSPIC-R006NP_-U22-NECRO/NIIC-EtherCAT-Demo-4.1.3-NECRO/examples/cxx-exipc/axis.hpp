#pragma once

#include <ecat/types.hpp>
#include <map>

struct axis_data
{
  std::uint16_t axis_id;
  std::uint16_t slave_pos;
  // bool tmp = false;


  // process data address
  volatile std::uint16_t *control_word;
  volatile std::int32_t *target_position;
  volatile std::int32_t *target_velocity;
  volatile std::int8_t *mode_of_operation;

  volatile const std::uint16_t *error_code;
  volatile const std::uint16_t *status_word;
  volatile const std::int32_t *position_actual_value;
  volatile const std::int32_t *velocity_actual_value;
  volatile const std::int8_t *mode_of_operation_display;

  int16_t control_word_buffer;
  int32_t target_position_buffer;
  int32_t target_velocity_buffer;
  int8_t mode_of_operation_buffer;

  uint16_t error_code_buffer;
  uint16_t status_word_buffer;
  int32_t position_actual_value_buffer;
  int32_t velocity_actual_value_buffer;
  int8_t mode_of_operation_display_buffer;

  volatile std::uint32_t *pv;
  volatile std::uint32_t *pa;
  volatile std::uint32_t *pd;
  int32_t p_buf;

  std::uint16_t old_status_word;
  int mode;
};
