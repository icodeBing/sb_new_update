// Ti5电机控制
#pragma once

#include "axis.hpp"
#include<sys/time.h>
#include "interface.hpp"

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

   
    if(axis->master_id == 1)
    {//主站1总线上所有电机的目标位置赋值和读取实际电机位置操作
        // 将ros2下发的目标位置赋值给*axis（实际对应的每个电机）
        *axis->target_position = motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_cmd; //csp

        // 读取电机实际位置和实际速度值
        motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_actual = *axis->position_actual_value;
        motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_actual = *axis->velocity_actual_value;
        // *axis->target_velocity = motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_cmd;  // csv
        // *axis->target_torque = 50  ;

        //  *axis->target_torque = 100;    //cst
          
    }
    if(axis->master_id == 2)
    {//主站2总线上所有电机的目标位置赋值和读取实际电机位置操作
        *axis->target_position = motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_cmd; //csp
        
        motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_actual = *axis->position_actual_value;
        motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_actual = *axis->velocity_actual_value;
    }

    // get_motor_position_actual_value[axis->axis_id] = *axis->position_actual_value;
    // get_motor_velocity_actual_value[axis->axis_id] = *axis->velocity_actual_value;
    // get_ethercat_master_id[axis->axis_id] = axis->master_id;
    // get_motor_axis_id[axis->axis_id] = axis->axis_id;

    /*****************************(csp)*******************************************/
    // *axis->target_position = *axis->position_actual_value +10000; //csps
    /***************************************************************************/

     /*****************************(cst)*******************************************/
    // *axis->target_torque =  20000; //cst
    /***************************************************************************/

    // std::cout<<"master_id:  "<<axis->master_id << std::endl;
    // std::cout<<"torque:    "<<*axis->torque_actual_value<<std::endl;
    // std::cout<<"position:  "<<*axis->position_actual_value<<std::endl;
    // std::cout<<"velocity:  "<<*axis->velocity_actual_value<<std::endl;

    error = false;
    errorid = 0;
  }
};
