#ifndef GLOBAL_PARAMS_H
#define GLOBAL_PARAMS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <atomic>
#include <vector>
#include <array>
#include <mutex>
#include "motor_interface/msg/joint_motor.hpp"

#define EXIPC_PORT_1 1        // IPC进程通信端口定义
#define motor_num 23          // 电机数量
#define ethercat_master_num 3 // 国讯ethercat主站个数
#define master_count 3        // 主站数量

// 电机消息结构体定义
struct Motor_msg_single
{
    int32_t kp_cmd;
    int32_t kd_cmd;
    int32_t Tor_cmd;
    int32_t position_cmd;
    int32_t velocity_cmd;
    int32_t position_actual;
    int32_t velocity_actual;
    int32_t current_actual;
};

struct Motor_id
{
    Motor_msg_single motor_id[motor_num]; // 定义23个电机信息结构体，每个电机信息包括指令值和实际值
};

struct Ethercat_master_diag
{
    int32_t wc_state;
    int32_t lost_frame_count;
    int32_t lost_frame_delta;
    int32_t slaves_responding;
    int32_t master_al_state;
    int32_t latency_flag;
    int32_t latency_max_us;
    int32_t latency_min_us;
    int32_t latency_avg_us;
    uint32_t cycle_counter;
};

struct Ethercat_comm_diag
{
    uint32_t version;
    uint32_t update_seq;
    Ethercat_master_diag master_diag[master_count];
};

struct Motor_master
{
    Motor_id master_id[master_count]; // 定义三个主站结构体，每个主站都有23个电机信息，但是实际每个主站可能用不到那么多电机
    Ethercat_comm_diag ec_diag;
};

enum Motor_Ctrl_Mode_type
{                  // 电机控制模式类型
    Pos_Mode,      // 位置控制模式
    Force_Pos_Mode // 力位混控模式
};

extern std::array<std::string, motor_num> joint_names;
extern std::mutex joint_mutex;
extern std::shared_ptr<motor_interface::msg::JointMotor> global_Joint_data;
extern std::shared_ptr<motor_interface::msg::JointMotor> temp_global_Joint_data;
extern std::mutex joint_mutex2;
extern std::shared_ptr<motor_interface::msg::JointMotor> global_Joint_data_pub;
extern std::shared_ptr<motor_interface::msg::JointMotor> temp_global_Joint_data_pub;
extern std::int8_t Motor_Index_Switch[motor_num];
#endif
