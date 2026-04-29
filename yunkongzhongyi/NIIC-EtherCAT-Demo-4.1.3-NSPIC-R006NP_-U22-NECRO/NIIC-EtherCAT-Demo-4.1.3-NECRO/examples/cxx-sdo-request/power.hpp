#pragma once

#include "axis.hpp"

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
