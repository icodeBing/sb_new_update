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
    // std::cout<<"error_code:  "<<*axis->error_code<<std::endl;
    
    
    //如果是第一次循环，会将当前实际位置赋值给初始指定位置
    if (!executing_in_process)
    {
      pos = *axis->position_actual_value;
      printf(".......... move start pos: %d\n.", *axis->position_actual_value);
    }
    executing_in_process = execute;


    
    // if(axis->axis_id == 0)
    // // {
      // *axis->target_position = *axis->position_actual_value +10000; //csp

      *axis->target_velocity = *axis->velocity_actual_value + 50000; //csv

    // }


    /*****************************(csp)*******************************************/
    // *axis->target_position = *axis->position_actual_value +10000; //csp
    /***************************************************************************/

     /*****************************(cst)*******************************************/
    // *axis->target_torque =  20000; //cst
    /***************************************************************************/




    

    // std::cout<<"torque: "<<*axis->torque_actual_value<<std::endl;
    // std::cout<<"    position"<<*axis->position_actual_value<<std::endl;




    error = false;
    errorid = 0;
  }
};
