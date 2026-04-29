#pragma once

#include "axis.hpp"

extern Motor_Ctrl_Mode_type Ti5_Motor_Ctrl_Mode;
class power
{
public:
  // in out
  axis_data *axis;
  
  // in
  bool enable;

  // out
  bool status;
  bool valid;
  bool error;
  int errorid;

  void on_cycle();

};
