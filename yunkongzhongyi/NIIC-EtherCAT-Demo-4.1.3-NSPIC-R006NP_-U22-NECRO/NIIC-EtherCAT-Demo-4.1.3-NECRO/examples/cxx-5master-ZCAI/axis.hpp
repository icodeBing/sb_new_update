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
  volatile std::int8_t *mode_of_operation;

  volatile const std::uint16_t *error_code;
  volatile const std::uint16_t *status_word;
  volatile const std::int32_t *position_actual_value;
  volatile const std::int8_t *mode_of_operation_display;

  std::uint16_t old_status_word;
};
