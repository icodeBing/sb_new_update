#pragma once

#include "axis.hpp"
#include "ServoIpcStruct.h"

class power
{
public:
  // in out
  axis_data *axis;
  // ServoControl_t servoControl;
  
  // in
  bool enable;

  // out
  bool status;
  bool valid;
  bool error;
  int errorid;

  void on_cycle();

};
