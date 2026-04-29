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
    executing_in_process = execute;

    // if (count < 100) //正向加速
    // {
    //   vel += 2;
    //   pos += vel;
    // }
    // else
    // {
    //   pos += vel;
    // }
    // count += 1;
    // if(count > 100000000)
    // {
    //   count = 101;
    // }
    // if(count % 1000 == 0)
    // {
    //   for(int i = 0; i < 1000; i++)
    //   {
    //     struct timeval tv;
    //     gettimeofday(&tv, nullptr);
    //   }
    // }


    *axis->target_position = *axis->position_actual_value+10000 ;

    error = false;
    errorid = 0;
  }
};
