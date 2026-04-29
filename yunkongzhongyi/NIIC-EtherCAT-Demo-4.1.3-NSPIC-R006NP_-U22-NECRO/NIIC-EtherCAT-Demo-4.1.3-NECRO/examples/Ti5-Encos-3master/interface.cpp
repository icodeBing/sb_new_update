#include "interface.hpp"
#include "ecat/niic_api.hpp"

#include <atomic>
#include <chrono>
#include <iostream>

namespace
{
    using SteadyClock = std::chrono::steady_clock;

    constexpr uint32_t kDiagVersion = 1;
    constexpr auto kDiagSamplePeriod = std::chrono::milliseconds(100);
    constexpr auto kDiagLogPeriod = std::chrono::seconds(1);

    ecat::task *g_registered_tasks[master_number] = {nullptr, nullptr, nullptr};
    std::atomic<int32_t> g_cycle_wc_state[master_number];
    std::atomic<uint32_t> g_cycle_counter[master_number];
    Ethercat_comm_diag g_cached_diag{};
    int32_t g_last_logged_wc_state[master_number] = {-2, -2, -2};

    const char *wc_state_to_string(int32_t wc_state)
    {
        switch (wc_state)
        {
        case static_cast<int32_t>(ecat::wc_state_type::zero):
            return "zero";
        case static_cast<int32_t>(ecat::wc_state_type::incomplete):
            return "incomplete";
        case static_cast<int32_t>(ecat::wc_state_type::complete):
            return "complete";
        default:
            return "unknown";
        }
    }

    void init_cached_diag()
    {
        memset(&g_cached_diag, 0, sizeof(g_cached_diag));
        g_cached_diag.version = kDiagVersion;
        for (int master_idx = 0; master_idx < master_number; ++master_idx)
        {
            g_cached_diag.master_diag[master_idx].wc_state = -1;
            g_cached_diag.master_diag[master_idx].lost_frame_count = -1;
            g_cached_diag.master_diag[master_idx].lost_frame_delta = 0;
            g_cached_diag.master_diag[master_idx].slaves_responding = -1;
            g_cached_diag.master_diag[master_idx].master_al_state = -1;
            g_cached_diag.master_diag[master_idx].latency_flag = 0;
            g_cached_diag.master_diag[master_idx].latency_max_us = -1;
            g_cached_diag.master_diag[master_idx].latency_min_us = -1;
            g_cached_diag.master_diag[master_idx].latency_avg_us = -1;
            g_cached_diag.master_diag[master_idx].cycle_counter = 0;
        }
    }

    void update_fast_diag_fields(Ethercat_comm_diag *diag)
    {
        diag->version = kDiagVersion;
        for (int master_idx = 0; master_idx < master_number; ++master_idx)
        {
            diag->master_diag[master_idx].wc_state =
                g_cycle_wc_state[master_idx].load(std::memory_order_relaxed);
            diag->master_diag[master_idx].cycle_counter =
                g_cycle_counter[master_idx].load(std::memory_order_relaxed);
        }
    }

    void sample_slow_diag_fields(Ethercat_comm_diag *diag, int32_t prev_lostframe[master_number])
    {
        ++diag->update_seq;
        for (int master_idx = 0; master_idx < master_number; ++master_idx)
        {
            auto &master_diag = diag->master_diag[master_idx];
            auto *task = g_registered_tasks[master_idx];
            if (task == nullptr)
            {
                master_diag.lost_frame_count = -1;
                master_diag.lost_frame_delta = 0;
                master_diag.slaves_responding = -1;
                master_diag.master_al_state = -1;
                master_diag.latency_flag = 0;
                master_diag.latency_max_us = -1;
                master_diag.latency_min_us = -1;
                master_diag.latency_avg_us = -1;
                continue;
            }

            int lostframe_count = 0;
            int slaves_responding = 0;
            uint8_t master_al_state = 0;
            ecat::RunTimeData latency{};

            ecat::get_lostframe_count(task, lostframe_count);
            ecat::get_slaves_responding(task, slaves_responding);
            ecat::get_Master_State(task, master_al_state);
            ecat::get_latency_in_task(task, &latency);

            master_diag.lost_frame_count = lostframe_count;
            if (prev_lostframe[master_idx] < 0)
            {
                master_diag.lost_frame_delta = 0;
            }
            else
            {
                master_diag.lost_frame_delta = lostframe_count - prev_lostframe[master_idx];
            }
            if (master_diag.lost_frame_delta < 0)
            {
                master_diag.lost_frame_delta = 0;
            }
            prev_lostframe[master_idx] = lostframe_count;

            master_diag.slaves_responding = slaves_responding;
            master_diag.master_al_state = static_cast<int32_t>(master_al_state);
            master_diag.latency_flag = static_cast<int32_t>(latency.flag);
            master_diag.latency_max_us = latency.maxDelay;
            master_diag.latency_min_us = latency.minDelay;
            master_diag.latency_avg_us = latency.avgDelay;
        }
        update_fast_diag_fields(diag);
    }

    void print_diag_summary(const Ethercat_comm_diag &diag, const char *tag)
    {
        printf("[Server][EtherCATDiag] %s: version=%u update_seq=%u\n",
               tag,
               diag.version,
               diag.update_seq);
        for (int master_idx = 0; master_idx < master_number; ++master_idx)
        {
            const auto &master_diag = diag.master_diag[master_idx];
            printf("[Server][EtherCATDiag] master%d wc=%s cycle=%u lost=%d delta=%d responding=%d "
                   "al_state=%d latency(us) min/avg/max=%d/%d/%d\n",
                   master_idx,
                   wc_state_to_string(master_diag.wc_state),
                   master_diag.cycle_counter,
                   master_diag.lost_frame_count,
                   master_diag.lost_frame_delta,
                   master_diag.slaves_responding,
                   master_diag.master_al_state,
                   master_diag.latency_min_us,
                   master_diag.latency_avg_us,
                   master_diag.latency_max_us);
        }
    }

    void warn_if_diag_abnormal(const Ethercat_comm_diag &diag)
    {
        for (int master_idx = 0; master_idx < master_number; ++master_idx)
        {
            const auto &master_diag = diag.master_diag[master_idx];
            if (master_diag.wc_state != g_last_logged_wc_state[master_idx])
            {
                if (master_diag.wc_state < 0)
                {
                    g_last_logged_wc_state[master_idx] = master_diag.wc_state;
                }
                else if (master_diag.wc_state != static_cast<int32_t>(ecat::wc_state_type::complete))
                {
                    fprintf(stderr,
                            "[Server][EtherCATDiag] master%d WKC异常，当前状态=%s\n",
                            master_idx,
                            wc_state_to_string(master_diag.wc_state));
                }
                else if (g_last_logged_wc_state[master_idx] != -2)
                {
                    fprintf(stdout,
                            "[Server][EtherCATDiag] master%d WKC恢复正常，当前状态=%s\n",
                            master_idx,
                            wc_state_to_string(master_diag.wc_state));
                }
                g_last_logged_wc_state[master_idx] = master_diag.wc_state;
            }
            if (master_diag.lost_frame_delta > 0)
            {
                // fprintf(stderr,
                //         "[Server][EtherCATDiag] master%d lost frame增加了%d，累计=%d\n",
                //         master_idx,
                //         master_diag.lost_frame_delta,
                //         master_diag.lost_frame_count);
            }
        }
    }
} // namespace

Motor_master motor_msg;

void fail(const char *reason)
{
    perror(reason);
    exit(EXIT_FAILURE);
}

// 转换Motor_msg网络序到主机序（仅处理指令字段，实际值字段忽略）
// 网络序→主机序（仅指令字段）
void motor_msg_ntohl(Motor_master *net_msg, Motor_master *host_msg)
{
    memset(host_msg, 0, sizeof(Motor_master));
    for (int m = 0; m < 3; m++)
    {
        for (int i = 0; i < motor_number; i++)
        {
            host_msg->master_id[m].motor_id[i].kp_cmd = ntohl(net_msg->master_id[m].motor_id[i].kp_cmd);
            host_msg->master_id[m].motor_id[i].kd_cmd = ntohl(net_msg->master_id[m].motor_id[i].kd_cmd);
            host_msg->master_id[m].motor_id[i].Tor_cmd = ntohl(net_msg->master_id[m].motor_id[i].Tor_cmd);
            host_msg->master_id[m].motor_id[i].position_cmd = ntohl(net_msg->master_id[m].motor_id[i].position_cmd);
            host_msg->master_id[m].motor_id[i].velocity_cmd = ntohl(net_msg->master_id[m].motor_id[i].velocity_cmd);
        }
    }

    // data process to double
    // host_msg->master_id[0].motor_id[4].Tor_cmd = host_msg->master_id[0].motor_id[4].Tor_cmd / 1000.0f;
    // host_msg->master_id[0].motor_id[5].Tor_cmd = host_msg->master_id[0].motor_id[5].Tor_cmd / 1000.0f;
}

// 主机序→网络序（仅实际值字段）
void motor_msg_htonl(Motor_master *host_msg, Motor_master *net_msg)
{
    memset(net_msg, 0, sizeof(Motor_master));
    for (int m = 0; m < 3; m++)
    {
        for (int i = 0; i < motor_number; i++)
        {
            net_msg->master_id[m].motor_id[i].position_actual = htonl(host_msg->master_id[m].motor_id[i].position_actual);
            net_msg->master_id[m].motor_id[i].velocity_actual = htonl(host_msg->master_id[m].motor_id[i].velocity_actual);
            net_msg->master_id[m].motor_id[i].current_actual = htonl(host_msg->master_id[m].motor_id[i].current_actual);
            // std::cout<<  "host:"<<host_msg->master_id[m].motor_id[i].current_actual<<  "      net:"<<net_msg->master_id[m].motor_id[i].current_actual<<std::endl;
        }
    }
    net_msg->ec_diag.version = htonl(host_msg->ec_diag.version);
    net_msg->ec_diag.update_seq = htonl(host_msg->ec_diag.update_seq);
    for (int m = 0; m < master_number; ++m)
    {
        net_msg->ec_diag.master_diag[m].wc_state = htonl(host_msg->ec_diag.master_diag[m].wc_state);
        net_msg->ec_diag.master_diag[m].lost_frame_count = htonl(host_msg->ec_diag.master_diag[m].lost_frame_count);
        net_msg->ec_diag.master_diag[m].lost_frame_delta = htonl(host_msg->ec_diag.master_diag[m].lost_frame_delta);
        net_msg->ec_diag.master_diag[m].slaves_responding = htonl(host_msg->ec_diag.master_diag[m].slaves_responding);
        net_msg->ec_diag.master_diag[m].master_al_state = htonl(host_msg->ec_diag.master_diag[m].master_al_state);
        net_msg->ec_diag.master_diag[m].latency_flag = htonl(host_msg->ec_diag.master_diag[m].latency_flag);
        net_msg->ec_diag.master_diag[m].latency_max_us = htonl(host_msg->ec_diag.master_diag[m].latency_max_us);
        net_msg->ec_diag.master_diag[m].latency_min_us = htonl(host_msg->ec_diag.master_diag[m].latency_min_us);
        net_msg->ec_diag.master_diag[m].latency_avg_us = htonl(host_msg->ec_diag.master_diag[m].latency_avg_us);
        net_msg->ec_diag.master_diag[m].cycle_counter = htonl(host_msg->ec_diag.master_diag[m].cycle_counter);
    }
}

// 将实际电机位置和实际电机速度值通过IPC通信给到ros2
void generate_actual_value(Motor_master *recv_cmd, Motor_master *send_actual)
{
    memset(send_actual, 0, sizeof(Motor_master));
    for (int m = 0; m < 3; m++)
    {
        for (int i = 0; i < motor_number; i++)
        {

            send_actual->master_id[m].motor_id[i].position_actual = motor_msg.master_id[m].motor_id[i].position_actual;
            send_actual->master_id[m].motor_id[i].velocity_actual = motor_msg.master_id[m].motor_id[i].velocity_actual;
            send_actual->master_id[m].motor_id[i].current_actual = motor_msg.master_id[m].motor_id[i].current_actual;
        }
    }
    send_actual->ec_diag = g_cached_diag;
}

// 打印通过exipc通信接收到的ros2下发的指令信息
void print_recv_cmd(const char *prefix, struct Motor_master *msg)
{
    // printf("%s:\n", prefix);
    // 遍历每个Master
    // for (int m = 0; m < 3; m++) {
    //     printf("  Master%d 位置指令(position_cmd)：", m);
    //     // 打印前5个电机，后续用...省略
    //     for (int i = 0; i < motor_number; i++) {
    //         printf("电机%d:%d ", i, msg->master_id[m].motor_id[i].position_cmd);
    //     }
    //     printf("\n");

    //     printf("  Master%d 速度指令(velocity_cmd)：", m);
    //     for (int i = 0; i < motor_number; i++) {
    //         printf("电机%d:%d ", i, msg->master_id[m].motor_id[i].velocity_cmd);
    //     }
    //     printf("\n");

    //     printf("  Master%d kp指令(kp_cmd)：", m);
    //     for (int i = 0; i < motor_number; i++) {
    //         printf("电机%d:%f ", i, msg->master_id[m].motor_id[i].kp_cmd);
    //     }
    //     printf("\n");

    //     printf("  Master%d kp指令(kd_cmd)：", m);
    //     for (int i = 0; i < motor_number; i++) {
    //         printf("电机%d:%f ", i, msg->master_id[m].motor_id[i].kd_cmd);
    //     }
    //     printf("\n");
    // }
    for (int m = 0; m < 3; m++)
    {
        for (int i = 0; i < motor_number; i++)
        {
            motor_msg.master_id[m].motor_id[i].kp_cmd = msg->master_id[m].motor_id[i].kp_cmd;
            motor_msg.master_id[m].motor_id[i].kd_cmd = msg->master_id[m].motor_id[i].kd_cmd;
            motor_msg.master_id[m].motor_id[i].Tor_cmd = msg->master_id[m].motor_id[i].Tor_cmd;
            motor_msg.master_id[m].motor_id[i].position_cmd = msg->master_id[m].motor_id[i].position_cmd;
            motor_msg.master_id[m].motor_id[i].velocity_cmd = msg->master_id[m].motor_id[i].velocity_cmd;
            // /****2026.1.22wk优化打印信息，省去重复的for循环**start**/
            // printf("  Master%d 电机%d 位置指令(position_cmd)：%d\n", m,i,motor_msg.master_id[m].motor_id[i].position_cmd);
            // printf("  Master%d 电机%d 速度指令(velocity_cmd)：%d\n", m,i,motor_msg.master_id[m].motor_id[i].velocity_cmd);
            // printf("  Master%d 电机%d Tor_cmd：%.4f\n", m,i,motor_msg.master_id[m].motor_id[i].Tor_cmd);
            // printf("  Master%d 电机%d kp：%.4f\n", m,i,motor_msg.master_id[m].motor_id[i].kp_cmd);
            // printf("  Master%d 电机%d kd：%.4f\n", m,i,motor_msg.master_id[m].motor_id[i].kd_cmd);
            // /****2026.1.22wk优化打印信息，省去重复的for循环**end**/
        }
    }

    // printf("--------------------------------------------------\n");
}

void print_send_actual(const char *prefix, struct Motor_master *msg)
{
    printf("%s:\n", prefix);
    // 遍历每个Master
    // for (int m = 0; m < 3; m++) {
    //     // printf("  Master%d 位置实际值(position_actual)：", m);
    //     // for (int i = 0; i < motor_number; i++) {
    //     //     printf("电机%d:%d ", i, msg->master_id[m].motor_id[i].position_actual);
    //     // }
    //     // printf("\n");

    //     // printf("  Master%d 速度实际值(velocity_actual)：", m);
    //     // for (int i = 0; i < motor_number; i++) {
    //     //     printf("电机%d:%d ", i, msg->master_id[m].motor_id[i].velocity_actual);
    //     // }
    //     // printf("\n");
    //     printf("  Master%d effort实际值(effort)：", m);
    //     for (int i = 0; i < motor_number; i++) {
    //         printf("电机%d:%.2f ", i, msg->master_id[m].motor_id[i].current_actual);
    //     }
    //     printf("\n");

    // }
    // printf("--------------------------------------------------\n");
}

void *server(void *)
{
    struct sockaddr_ipc saddr, claddr;
    socklen_t addrlen = sizeof(claddr);
    struct Motor_master net_recv_msg, host_recv_cmd;    // 接收缓冲区（网络序/主机序指令）
    struct Motor_master host_send_actual, net_send_msg; // 发送缓冲区（主机序实际值/网络序）

    size_t poolsz;
    int ret, sfd = -1;
    struct timespec ts;
    srand(time(NULL)); // 初始化随机数种子（用于模拟实际值误差）
    auto last_diag_sample = SteadyClock::now();
    auto last_diag_log = last_diag_sample;
    int32_t prev_lostframe[master_number] = {-1, -1, -1};

    init_cached_diag();

    // 创建SOCK_DGRAM socket
    sfd = __RT(socket(AF_RTIPC, SOCK_DGRAM, IPCPROTO_EXIPC));

    // 设置接收缓冲区大小（支持10个缓冲包）
    poolsz = BUF_SIZE * 10;
    ret = __RT(setsockopt(sfd, SOL_EXIPC, EXIPC_POOLSZ, &poolsz, sizeof(poolsz)));
    if (ret < 0)
    {
        close(sfd); // 失败时关闭sfd，释放已创建的socket资源
        fail("设置接收缓冲区失败");
    }

    // 绑定端口
    memset(&saddr, 0, sizeof(saddr));
    saddr.sipc_family = AF_RTIPC;
    saddr.sipc_port = EXIPC_PORT_1;
    ret = __RT(bind(sfd, (struct sockaddr *)&saddr, sizeof(saddr)));
    if (ret < 0)
    {
        close(sfd); // 绑定失败时关闭sfd，释放socket并解绑定端口
        fail("端口绑定失败");
    }

    printf("[Server] 启动成功（端口：%d）！\n", EXIPC_PORT_1);
    printf("[Server] 功能：接收客户端指令 → 反馈位置/速度实际值（结构体大小：%zu字节）\n",
           BUF_SIZE);

    for (;;)
    {
        const auto now = SteadyClock::now();
        if (now - last_diag_sample >= kDiagSamplePeriod)
        {
            sample_slow_diag_fields(&g_cached_diag, prev_lostframe);
            warn_if_diag_abnormal(g_cached_diag);
            last_diag_sample = now;
        }
        else
        {
            update_fast_diag_fields(&g_cached_diag);
        }

        if (now - last_diag_log >= kDiagLogPeriod)
        {
            // print_diag_summary(g_cached_diag, "周期汇总");
            last_diag_log = now;
        }

        // 1. 接收客户端发送的指令（网络序）
        ret = __RT(recvfrom(sfd, &net_recv_msg, BUF_SIZE, 0,
                            (struct sockaddr *)&claddr, &addrlen));

        if (ret == BUF_SIZE)
        {
            // 2. 网络序 → 主机序（仅解析指令字段）

            // std::cout << "tor_cmd_4:before:" << net_recv_msg.master_id[0].motor_id[4].Tor_cmd <<std::endl;

            motor_msg_ntohl(&net_recv_msg, &host_recv_cmd);
            // printf("[Server] 收到客户端指令！\n");
            print_recv_cmd("[Server] 接收的指令", &host_recv_cmd);

            // 3. 根据指令生成实际值（用户自定义逻辑）
            generate_actual_value(&host_recv_cmd, &host_send_actual);

            // 4. 主机序 → 网络序（仅转换实际值字段）
            motor_msg_htonl(&host_send_actual, &net_send_msg);

            // 5. 发送实际值给客户端
            ret = __RT(sendto(sfd, &net_send_msg, BUF_SIZE, 0,
                              (struct sockaddr *)&claddr, addrlen));

            if (ret == BUF_SIZE)
            {
                // printf("[Server] 实际值反馈成功！\n");
                // print_send_actual("[Server] 反馈的实际值", &host_send_actual);
            }
            else
            {
                printf("[Server] 实际值反馈失败：实际发送%d字节\n", ret);
            }
        }
        else if (ret > 0)
        {
            printf("[Server] 指令接收不完整：实际接收%d字节 / 需接收%zu字节\n", ret, BUF_SIZE);
        }
        else
        {
            perror("[Server] 指令接收失败");
            close(sfd);
            fail("recvfrom错误");
        }

        // 1ms延时，降低CPU占用
        ts.tv_sec = 0;
        ts.tv_nsec = 1000000;
        clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    }

    close(sfd);
    return NULL;
}

void interface_register_tasks(ecat::task *master0, ecat::task *master1, ecat::task *master2)
{
    g_registered_tasks[0] = master0;
    g_registered_tasks[1] = master1;
    g_registered_tasks[2] = master2;
    for (int master_idx = 0; master_idx < master_number; ++master_idx)
    {
        g_cycle_wc_state[master_idx].store(-1, std::memory_order_relaxed);
        g_cycle_counter[master_idx].store(0, std::memory_order_relaxed);
        g_last_logged_wc_state[master_idx] = -2;
    }
    init_cached_diag();
}

void update_master_cycle_status(int master_index, int wc_state)
{
    if (master_index < 0 || master_index >= master_number)
    {
        return;
    }
    g_cycle_wc_state[master_index].store(wc_state, std::memory_order_relaxed);
    g_cycle_counter[master_index].fetch_add(1, std::memory_order_relaxed);
}

// 线程配置（保持原实时属性）
void interface_task()
{
    pthread_t p2;
    pthread_attr_t ap2;
    struct sched_param sp2;
    cpu_set_t cp2;

    memset(&ap2, 0, sizeof(pthread_attr_t));
    memset(&sp2, 0, sizeof(struct sched_param));
    memset(&cp2, 0, sizeof(cpu_set_t));

    pthread_attr_init(&ap2);
    pthread_attr_setinheritsched(&ap2, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&ap2, SCHED_FIFO);
    sp2.sched_priority = 50;
    pthread_attr_setschedparam(&ap2, &sp2);
    CPU_ZERO(&cp2);
    CPU_SET(4, &cp2);
    pthread_attr_setaffinity_np(&ap2, sizeof(cp2), &cp2);

    __RT(pthread_create(&p2, &ap2, server, NULL));
    __RT(pthread_setname_np(p2, "motor-server-actual"));

    pthread_attr_destroy(&ap2);
    // __RT(pthread_join(p2, NULL));
    pthread_detach(p2); // 替代 pthread_join()，主线程不阻塞
}
