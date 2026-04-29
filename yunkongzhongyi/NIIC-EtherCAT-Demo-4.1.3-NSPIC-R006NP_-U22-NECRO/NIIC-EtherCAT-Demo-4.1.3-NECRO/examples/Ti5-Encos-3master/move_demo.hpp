// Ti5电机控制
#pragma once

#include "axis.hpp"
#include <sys/time.h>
#include "interface.hpp"
#include "power.hpp"
// 定义型号
#define ACTUATOR_TYPE CRA_RI110170_PRO

#define KP_MAX 500.0f
#define KP_MIN 0.0f
#define KD_MAX 5.0f
#define KD_MIN 0.0f
#define MAX_VEL 18.0f
#define MIN_VEL -18.0f
#define MAX_POS 12.5f
#define MIN_POS -12.5f

#define T_MAX_CRA_RI3040_PRO 3.3f
#define T_MIN_CRA_RI3040_PRO -3.3f
#define T_MAX_CRA_RI4052_PRO 8.3f
#define T_MIN_CRA_RI4052_PRO -8.3f
#define T_MAX_CRA_RI5060_PRO 23.0f
#define T_MIN_CRA_RI5060_PRO -23.0f
#define T_MAX_CRA_RI6070_PRO 42.0f
#define T_MIN_CRA_RI6070_PRO -42.0f

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

        // 当状态切换到operation_enabled的时候，execute才会被置为true
        if (!execute)
        {
            valid = false;
            return;
        }
        // std::cout<<"error_code:  "<<*axis->error_code<<std::endl;

        // 如果是第一次循环，会将当前实际位置赋值给初始指定位置
        if (!executing_in_process)
        {
            if (Ti5_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
            { // 位置控制模式
                pos = *axis->position_actual_value;
                printf(".......... move start pos: %d\n.", *axis->position_actual_value);
            }
            else if (Ti5_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
            { // 力位混控模式
                pos = *axis->actual_pos;
                printf(".......... move start pos: %d\n.", *axis->actual_pos);
            }
        }
        executing_in_process = execute;

        if (axis->master_id == 1 || axis->master_id == 2)
        { // 主站1总线上所有电机的目标位置赋值和读取实际电机位置操作
            // 将ros2下发的目标位置赋值给*axis（实际对应的每个电机）
            if (Ti5_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
            {                                                                           
                // printf("pos:%d\n",motor_msg.master_id[1].motor_id[0].position_cmd);                            // 位置控制模式
                *axis->target_position = motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_cmd; // csp
                // 读取电机实际位置和实际速度值
                motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_actual = *axis->position_actual_value;
                motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_actual = *axis->velocity_actual_value;
            }
            else if (Ti5_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
            {                        
                                                                                                    // 力位混控模式
                float temp_kp = (float)(motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].kp_cmd) / 10000.0f; // ros下发过来的是int32_t数据，放大了10000倍，所以这里缩小10000倍，转换成浮点数
                float temp_kd = (float)(motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].kd_cmd) / 10000.0f;
                float temp_target_pos = (float)(motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_cmd) / 10000.0f; // (rad)
                float temp_target_ver = (float)(motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_cmd) / 10000.0f; // (rad/s)
                float temp_target_tor = (float)(motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].Tor_cmd) / 10000.0f;   // (A)
                // 限幅
                temp_kp = get_check_data(temp_kp, KP_MAX, KP_MIN);
                temp_kd = get_check_data(temp_kd, KD_MAX, KD_MIN);
                temp_target_pos = get_check_data(temp_target_pos, MAX_POS, MIN_POS);
                temp_target_ver = get_check_data(temp_target_ver, MAX_VEL, MIN_VEL);
                if (axis->master_id == 1)
                { // 主站1负责脖子+左臂四个电机
                    if (axis->axis_id == 0)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI4052_PRO, T_MIN_CRA_RI4052_PRO);
                    else if (axis->axis_id == 1)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI5060_PRO, T_MIN_CRA_RI5060_PRO);
                    else if (axis->axis_id == 2)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI4052_PRO, T_MIN_CRA_RI4052_PRO);
                    else if (axis->axis_id == 3)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI4052_PRO, T_MIN_CRA_RI4052_PRO);
                    else if (axis->axis_id == 4)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI5060_PRO, T_MIN_CRA_RI5060_PRO);
                }
                else if (axis->master_id == 2)
                { // 主站2负责腰部两个电机+右臂四个电机
                    if (axis->axis_id == 0)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI5060_PRO, T_MIN_CRA_RI5060_PRO);
                    else if (axis->axis_id == 1)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI4052_PRO, T_MIN_CRA_RI4052_PRO);
                    else if (axis->axis_id == 2)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI4052_PRO, T_MIN_CRA_RI4052_PRO);
                    else if (axis->axis_id == 3)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI5060_PRO, T_MIN_CRA_RI5060_PRO);
                    else if (axis->axis_id == 4)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI6070_PRO, T_MIN_CRA_RI6070_PRO);
                    else if (axis->axis_id == 5)
                        temp_target_tor = get_check_data(temp_target_tor, T_MAX_CRA_RI6070_PRO, T_MIN_CRA_RI6070_PRO);
                }

                // 赋值给*axis
                *axis->kp = temp_kp;
                *axis->kd = temp_kd;
                *axis->target_pos = temp_target_pos;
                *axis->actual_vel = temp_target_ver;
                *axis->target_tor = temp_target_tor;

                // if(axis->master_id == 1 && axis->axis_id == 1)
                // {
                //     printf("kp:%.4f\n",temp_kp);                            
                //     printf("kd:%.4f\n",temp_kd);                   
                //     printf("actual_pos:%.4f\n",*axis->actual_pos);
                //     printf("target_pos:%.4f\n",temp_target_pos);                            
                //     printf("temp_target_ver:%.4f\n",temp_target_ver);                            
                //     printf("temp_target_tor:%.4f\n",temp_target_tor);
                // }

                // *axis->kp = 50;
                // *axis->kd = 2;
                // *axis->target_pos = *axis->actual_pos + 0.1;

                // 读取电机实际位置和实际速度值
                float position_actual = *axis->actual_pos; // (rad)
                float velocity_actual = *axis->actual_vel; // (rad/s)
                float current_actual = *axis->actual_cur;  // (A)

                // printf("master: %d , motor_id : %d , actual_pos: %.4f, target_pos: %.4f\n ******* \n",axis->master_id,axis->axis_id,*axis->actual_pos,temp_target_pos);
                

                // 统一放大10000倍，转换成int32_t数据
                motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_actual = static_cast<int32_t>(position_actual * 10000.0f);
                motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_actual = static_cast<int32_t>(velocity_actual * 10000.0f);
                motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].current_actual =  static_cast<int32_t>(current_actual * 10000.0f);
                // printf("int: %d\n",motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_actual);
            }
        }
        // if(axis->master_id == 2)
        // {//主站2总线上所有电机的目标位置赋值和读取实际电机位置操作
        //     *axis->target_position = motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_cmd; //csp

        //     motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].position_actual = *axis->position_actual_value;
        //     motor_msg.master_id[axis->master_id].motor_id[axis->axis_id].velocity_actual = *axis->velocity_actual_value;
        // }

        // std::cout<<"master_id:  "<<axis->master_id << std::endl;
        // std::cout<<"torque:    "<<*axis->torque_actual_value<<std::endl;
        // std::cout<<"position:  "<<*axis->position_actual_value<<std::endl;
        // std::cout<<"velocity:  "<<*axis->velocity_actual_value<<std::endl;

        error = false;
        errorid = 0;
    }

    float get_check_data(float data, float max_data, float min_data)
    { // 限幅函数
        if (data > max_data)
        {
            return max_data;
        }
        else if (data < min_data)
        {
            return min_data;
        }
        else
        {
            return data;
        }
    }
};