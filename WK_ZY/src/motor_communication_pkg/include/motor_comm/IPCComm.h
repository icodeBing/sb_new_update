#ifndef IPC_COMM_H
#define IPC_COMM_H

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <rldf/ipc.h>
#include <qiuniu/init.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <arpa/inet.h>
#include <atomic>
#include <vector>
#include <iostream>
#include "global/golbalParams.h"

#define RECV_BUF_SIZE sizeof(struct Motor_master)

class IPCComm
{
public:
    IPCComm();
    ~IPCComm();

    void stop();

private:
    void init_thread();
    static void *thread_fun(void *arg);
    void run();
    void send_zero_command();

private:
    // 初始化函数
    void init_motor_msg(struct Motor_master *msg);

    // 字节序转换函数
    void motor_msg_htonl(struct Motor_master *host_msg, struct Motor_master *net_msg);
    void motor_msg_ntohl(struct Motor_master *net_msg, struct Motor_master *host_msg);

    // 打印函数声明
    void print_send_cmd(const char *prefix, struct Motor_master *msg);
    void print_recv_actual(const char *prefix, struct Motor_master *msg);

    void recv_actual(struct Motor_master *msg);
    void check_startup_zero_offset(uint64_t reconnects,
                                   bool has_server_diag,
                                   const Ethercat_comm_diag &diag);

    double encos_pulse_to_effort(int32_t current_actual, int32_t encos_motor_idx);

    int32_t encos_effort_to_pulse(double current_actual);
    double encos_pulse_to_rad(int32_t position_actual); // 根据英克斯电机返回报文协议，进行数据转换
    double encos_pulse_to_angular_velocity(int32_t velocity_actual);
    double ti5_pulse_to_rad(int32_t position_actual);
    double ti5_pulse_to_angular_velocity(int32_t velocity_actual);

    int32_t encos_kp_to_pulse(double kp);
    int32_t encos_kd_to_pulse(double kd);

    int32_t encos_rad_to_degree(double rad);
    int32_t encos_angular_velocity_to_rad_per_min(double angular_velocity);
    int32_t ti5_rad_to_pulse(double rad);
    int32_t ti5_angular_velocity_to_pulse(double angular_velocity);

private:
    std::atomic<bool> g_stop;
    int fd_;
    bool startup_zero_offset_checked_ = false;
    // 线程配置
    pthread_t p1;                                                                  // 定义一个线程标识符p1,用于标识一个线程
    pthread_attr_t ap1;                                                            // 定义一个线程属性对象ap1，用于设置线程的属性
    struct sched_param sp1;                                                        // 定义一个调度参数结构体对象sp1，用于设置线程的cpu亲和性
    cpu_set_t cp1;                                                                 // 定义一个cpu集合对象cpl，用于设置线程的cpu亲和性
    Motor_Ctrl_Mode_type m_Motor_Ctrl_Mode = Motor_Ctrl_Mode_type::Force_Pos_Mode; // 默认力位混控模式，如果需要调整电机控制模式，修改改变量即可
    std::mutex ipc_joint_mutex2;                                                   // 仅IPC通信线程使用，独立锁
};

#endif
