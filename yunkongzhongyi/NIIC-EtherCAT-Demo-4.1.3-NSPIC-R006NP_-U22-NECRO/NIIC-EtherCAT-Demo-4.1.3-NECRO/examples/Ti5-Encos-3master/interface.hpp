#ifndef INTERFACE_HPP // 头文件保护宏，防止重复包含
#define INTERFACE_HPP

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <rldf/ipc.h>
#include <qiuniu/init.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <arpa/inet.h>

#define EXIPC_PORT_1 1
#define motor_number 23
#define master_number 3

namespace ecat
{
    class task;
}

// 电机消息结构体定义（服务器：收指令，发实际值）
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
    Motor_msg_single motor_id[motor_number]; // 12 B，紧凑
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
    Ethercat_master_diag master_diag[master_number];
};

struct Motor_master
{
    Motor_id master_id[master_number];
    Ethercat_comm_diag ec_diag;
};

extern Motor_master motor_msg;

// 缓冲区大小 = 结构体大小
#undef BUF_SIZE
#define BUF_SIZE sizeof(Motor_master)

extern void fail(const char *reason);

// 转换Motor_msg网络序到主机序（仅处理指令字段，实际值字段忽略）
extern void motor_msg_ntohl(struct Motor_master *net_msg, struct Motor_master *host_msg);
// 转换Motor_msg主机序到网络序（仅处理实际值字段，指令字段置0）
extern void motor_msg_htonl(struct Motor_master *host_msg, struct Motor_master *net_msg);
// 服务器根据接收的指令，生成对应的实际值（用户可自定义逻辑）
// 输入：客户端的指令（position_cmd/velocity_cmd）
// 输出：要反馈给客户端的实际值（position_actual/velocity_actual）
extern void generate_actual_value(struct Motor_master *recv_cmd, struct Motor_master *send_actual);

// 打印服务器接收的指令（仅打印position_cmd和velocity_cmd）
extern void print_recv_cmd(const char *prefix, struct Motor_master *msg);
// 打印服务器发送的实际值（仅打印position_actual_value和velocity_actual_value）
extern void print_send_actual(const char *prefix, struct Motor_master *msg);
// 服务器：接收指令（position_cmd/velocity_cmd），发送实际值（position_actual/velocity_actual）
extern void *server(void *);

// 线程配置（保持原实时属性）
extern void interface_register_tasks(ecat::task *master0, ecat::task *master1, ecat::task *master2);
extern void update_master_cycle_status(int master_index, int wc_state);
extern void interface_task();

#endif
