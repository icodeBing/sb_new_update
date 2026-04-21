#pragma once

#include "axis.hpp"
#include<sys/time.h>

struct move_demo
{
  // in out
  axis_data *axis;

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
    // if(tmp)
    // {
    //   return;
    // }
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
      if (count < 100) //正向加速
      {
        vel += 10;
        pos += vel;
      }
      else
      {
        pos += vel;
      }
      count += 1;
      if(count > 100000000)
      {
        count = 101;
      }
      // if(count % 1000 == 0)
      // {
      //   for(int i = 0; i < 1000; i++)
      //   {
      //     struct timeval tv;
      //     gettimeofday(&tv, nullptr);
      //   }
      // }


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
};
