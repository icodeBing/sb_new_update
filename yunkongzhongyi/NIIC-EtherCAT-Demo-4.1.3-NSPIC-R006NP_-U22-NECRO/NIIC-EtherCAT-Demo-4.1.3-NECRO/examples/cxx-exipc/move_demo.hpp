#pragma once

#include "axis.hpp"
#include <sys/time.h>

struct move_demo
{
  // in out
  axis_data *axis;
  ServoControl_t servoControl;
  ServoSta_t servoSta;
  // in
  bool execute;

  // out
  bool valid;
  bool error;
  int errorid;

  // local
  bool executing_in_process = 0;
  uint64_t count = 0;
  uint32_t vel = 0;
  int32_t pos = 0;
  bool tmp = false;
  int ppp = 10 * 8388608;

  void on_cycle()
  {
    //当状态切换到operation_enabled的时候，execute才会被置为true
    if (!execute)
    {
      valid = false;
      return;
    }
    
    //如果是第一次循环，会将当前实际位置赋值给初始指定位置
    if (!executing_in_process)
    {
      pos = *axis->position_actual_value;
      printf(".......... move start pos: %d\n.", *axis->position_actual_value);
    }
    
    if (axis->mode == 8)
    {
      count++;
      if (count < 100 || count >= 500 && count < 600)
      {
        vel += 10;
        pos += vel;
        count += 1;
      }
      else if ((count >= 100 && count < 200) ||
              (count >= 400 && count < 500))
      {
        count += 1;
        pos += vel;
      }
      else if (count >= 200 && count < 400)
      {
        vel -= 10;
        pos += vel;
        count += 1;
      }

      count %= 600;


      *axis->target_position = pos;
    }

    if (axis->mode == 1) //  pp_mode
    {
      count++;
      if (!executing_in_process)
      {
        *axis->control_word = 0xf;
        *axis->target_position = ppp;
        *axis->pv = 10 * 8388608;
        *axis->pa = 20 * 8388608;
        *axis->pd = 20 * 8388608;
        executing_in_process = execute;
        return;
      }

      if (count == 1)
      {
        ppp *= -1;
        *axis->target_position = ppp;
        *axis->control_word |= 0x10;
      }
      // if (count == 2)
      // {
      //   *axis->control_word |= 0x30;
      // }
      if (count == 2)
      {
        *axis->control_word &= ~(1 << 4);
      }
      if ((*axis->status_word >> 10) == 1 && count > 6)
      {
        count = 0;
      }
    }   

    error = false;
    errorid = 0;
    executing_in_process = execute;
  }

  void qt_on_cycle()
  {
    if (!execute)
    {
      valid = false;
      return;
    }
    axis->target_velocity_buffer= 1310720;
    if (*axis->position_actual_value != servoControl.targetPosition){
      int d = servoControl.targetPosition - *axis->position_actual_value;
      if (abs(d) > 10000){
        axis->p_buf = d > 0 ? 1000 : -1000;
        *axis->target_position += axis->p_buf;
    }
      else {
        *axis->target_position = servoControl.targetPosition;
      }
      
    }
    else{
      axis->p_buf = 0;
      *axis->target_position = *axis->position_actual_value;
    }
    servoSta.axisCount = axis->axis_id;
    servoSta.errorCode = *axis->error_code;
    servoSta.statusWord = *axis->status_word;
    servoSta.curMode = *axis->mode_of_operation_display;
    servoSta.curPos = *axis->position_actual_value;
    servoSta.curVel = *axis->velocity_actual_value;
  }


};
