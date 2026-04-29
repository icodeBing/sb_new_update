#include "motor_comm/IPCComm.h"
#include "motor_comm/logRecord.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <time.h>

#include "probe/single_shot_probe.h"

namespace
{
using SteadyClock = std::chrono::steady_clock;
using Microseconds = std::chrono::microseconds;

constexpr auto kCycleBudget = Microseconds(1000);
constexpr auto kReportPeriod = std::chrono::seconds(1);
constexpr int kFailureWarnThreshold = 5;
constexpr int kStaleFeedbackWarnThreshold = 20;
constexpr auto kSevereOverrunThreshold = Microseconds(2000);
constexpr int kNullJointDataLogPeriod = 1000;
constexpr double kStartupZeroOffsetWarnThreshold = 1e-1;

void sleep_for_cycle_budget()
{
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = static_cast<long>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(kCycleBudget).count());
    clock_nanosleep(CLOCK_REALTIME, 0, &ts, nullptr);
}
} // namespace

using motor_comm_log::comm_logger;
using motor_comm_log::command_packet_changed;
using motor_comm_log::CommMonitor;
using motor_comm_log::feedback_packet_changed;
using motor_comm_log::make_event_record;
using motor_comm_log::make_snapshot_record;
using motor_comm_log::reset_period_counters;
using motor_comm_log::server_diag_has_issue;
using motor_comm_log::to_us;

IPCComm::IPCComm()
{
    g_stop.store(false, std::memory_order_release);
    fd_ = -1;
    if (!comm_logger().start())
    {
        std::fprintf(stderr, "[Client][CommLog] CSV logger init failed\n");
    }
    init_thread();
}

IPCComm::~IPCComm()
{
    comm_logger().stop();
}

void IPCComm::init_thread()
{
    memset(&ap1, 0, sizeof(pthread_attr_t));
    memset(&sp1, 0, sizeof(struct sched_param));
    memset(&cp1, 0, sizeof(cpu_set_t));

    pthread_attr_init(&ap1);
    pthread_attr_setinheritsched(&ap1, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&ap1, SCHED_FIFO);
    sp1.sched_priority = 50;
    pthread_attr_setschedparam(&ap1, &sp1);
    CPU_ZERO(&cp1);
    CPU_SET(4, &cp1);
    pthread_attr_setaffinity_np(&ap1, sizeof(cp1), &cp1);

#ifdef __RT
    __RT(pthread_create(&p1, &ap1, &IPCComm::thread_fun, this));
    __RT(pthread_setname_np(p1, "motor-client-cmd"));
#else
    pthread_create(&p1, &ap1, &IPCComm::thread_fun, this);
    pthread_setname_np(p1, "motor-client-cmd");
#endif
    pthread_attr_destroy(&ap1);
}

void *IPCComm::thread_fun(void *arg)
{
    IPCComm *self = static_cast<IPCComm *>(arg);
    if (self != nullptr)
    {
        self->run();
    }
    return nullptr;
}

void IPCComm::stop()
{
    g_stop.store(true, std::memory_order_relaxed);

    send_zero_command();

    const int ret = pthread_join(p1, nullptr);
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
    if (ret != 0)
    {
        comm_logger().enqueue_event(make_event_record(
            "client_join_failed", ret, 0, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
    }
    comm_logger().stop();
}

void IPCComm::run()
{
    char *devname = nullptr;
    int ret = 0;
    struct Motor_master host_send_msg, net_send_msg;
    struct Motor_master net_recv_msg, host_recv_msg;
    CommMonitor monitor;
    bool has_server_diag = false;
    Ethercat_comm_diag last_server_diag{};
    int null_joint_data_cycles = 0;

    if (asprintf(&devname, "/dev/rtp%d", EXIPC_PORT_1) < 0)
    {
        comm_logger().enqueue_event(make_event_record(
            "device_name_alloc_failed", -1, errno, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
        return;
    }

    fd_ = open(devname, O_RDWR);
    free(devname);
    if (fd_ < 0)
    {
        comm_logger().enqueue_event(make_event_record(
            "device_open_failed", -1, errno, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
        return;
    }
    comm_logger().enqueue_event(make_event_record(
        "client_started", fd_, 0, 0, 0, 0, 0, false, Ethercat_comm_diag{}));

    while (!g_stop.load(std::memory_order_acquire))
    {
        const auto cycle_begin = SteadyClock::now();
        ++monitor.cycles;

        {
            std::lock_guard<std::mutex> lock(joint_mutex);
            if (global_Joint_data == nullptr)
            {
                ++null_joint_data_cycles;
            }
            else
            {
                temp_global_Joint_data = global_Joint_data;
                null_joint_data_cycles = 0;
            }
        }

        if (null_joint_data_cycles > 0)
        {
            if (null_joint_data_cycles == 1 || null_joint_data_cycles % kNullJointDataLogPeriod == 0)
            {
                comm_logger().enqueue_event(make_event_record(
                    "joint_data_null",
                    0,
                    0,
                    null_joint_data_cycles,
                    0,
                    0,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
            sleep_for_cycle_budget();
            continue;
        }

        init_motor_msg(&host_send_msg);

        for (int i = 0; i < 5; i++)
        {
            if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
            {
                host_send_msg.master_id[1].motor_id[i].position_cmd =
                    ti5_rad_to_pulse(temp_global_Joint_data->position[Motor_Index_Switch[i + 12]]);
                host_send_msg.master_id[1].motor_id[i].velocity_cmd =
                    ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[Motor_Index_Switch[i + 12]]);
            }
            else if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
            {
                host_send_msg.master_id[1].motor_id[i].kp_cmd =
                    float_to_int(temp_global_Joint_data->kp[Motor_Index_Switch[i + 12]]);
                host_send_msg.master_id[1].motor_id[i].kd_cmd =
                    float_to_int(temp_global_Joint_data->kd[Motor_Index_Switch[i + 12]]);
                host_send_msg.master_id[1].motor_id[i].Tor_cmd =
                    float_to_int(temp_global_Joint_data->effort[Motor_Index_Switch[i + 12]]);
                host_send_msg.master_id[1].motor_id[i].position_cmd =
                    float_to_int(temp_global_Joint_data->position[Motor_Index_Switch[i + 12]]);
                host_send_msg.master_id[1].motor_id[i].velocity_cmd =
                    float_to_int(temp_global_Joint_data->velocity[Motor_Index_Switch[i + 12]]);
            }
        }

        for (int i = 0; i < 6; i++)
        {
            if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
            {
                host_send_msg.master_id[2].motor_id[i].position_cmd =
                    ti5_rad_to_pulse(temp_global_Joint_data->position[Motor_Index_Switch[i + 17]]);
                host_send_msg.master_id[2].motor_id[i].velocity_cmd =
                    ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[Motor_Index_Switch[i + 17]]);
            }
            else if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
            {
                host_send_msg.master_id[2].motor_id[i].kp_cmd =
                    float_to_int(temp_global_Joint_data->kp[Motor_Index_Switch[i + 17]]);
                host_send_msg.master_id[2].motor_id[i].kd_cmd =
                    float_to_int(temp_global_Joint_data->kd[Motor_Index_Switch[i + 17]]);
                host_send_msg.master_id[2].motor_id[i].Tor_cmd =
                    float_to_int(temp_global_Joint_data->effort[Motor_Index_Switch[i + 17]]);
                host_send_msg.master_id[2].motor_id[i].position_cmd =
                    float_to_int(temp_global_Joint_data->position[Motor_Index_Switch[i + 17]]);
                host_send_msg.master_id[2].motor_id[i].velocity_cmd =
                    float_to_int(temp_global_Joint_data->velocity[Motor_Index_Switch[i + 17]]);
            }
        }

        for (int i = 0; i < 12; i++)
        {
            host_send_msg.master_id[0].motor_id[i].kp_cmd =
                encos_kp_to_pulse(temp_global_Joint_data->kp[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].kd_cmd =
                encos_kd_to_pulse(temp_global_Joint_data->kd[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].Tor_cmd =
                encos_effort_to_pulse(temp_global_Joint_data->effort[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].position_cmd =
                encos_rad_to_degree(temp_global_Joint_data->position[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].velocity_cmd =
                encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[Motor_Index_Switch[i]]);
        }

        const bool command_changed =
            monitor.has_prev_cmd && command_packet_changed(host_send_msg, monitor.prev_cmd);

        motor_msg_htonl(&host_send_msg, &net_send_msg);

        const auto tx_begin = SteadyClock::now();
        ret = write(fd_, &net_send_msg, sizeof(struct Motor_master));
        const auto tx_end = SteadyClock::now();
        monitor.max_tx_us = std::max(monitor.max_tx_us, to_us(tx_end - tx_begin));

        if (ret == sizeof(struct Motor_master))
        {
            ++monitor.tx_ok;
            monitor.consecutive_tx_failures = 0;
        }
        else if (ret < 0)
        {
            ++monitor.tx_fail;
            ++monitor.consecutive_tx_failures;
            if (errno != EAGAIN && errno != EWOULDBLOCK)
            {
                comm_logger().enqueue_event(make_event_record(
                    "tx_failed",
                    ret,
                    errno,
                    monitor.consecutive_tx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));

                char *reconnect_devname = nullptr;
                if (asprintf(&reconnect_devname, "/dev/rtp%d", EXIPC_PORT_1) >= 0)
                {
                    if (fd_ >= 0)
                    {
                        close(fd_);
                    }
                    fd_ = open(reconnect_devname, O_RDWR);
                    free(reconnect_devname);
                    ++monitor.reconnects;
                    comm_logger().enqueue_event(make_event_record(
                        fd_ < 0 ? "reconnect_failed" : "reconnect_success",
                        fd_,
                        fd_ < 0 ? errno : 0,
                        monitor.consecutive_tx_failures,
                        0,
                        monitor.stale_feedback_cycles,
                        monitor.reconnects,
                        has_server_diag,
                        last_server_diag));
                }
            }
        }
        else
        {
            ++monitor.tx_partial;
            ++monitor.consecutive_tx_failures;
            comm_logger().enqueue_event(make_event_record(
                "tx_partial",
                ret,
                0,
                monitor.consecutive_tx_failures,
                0,
                monitor.stale_feedback_cycles,
                monitor.reconnects,
                has_server_diag,
                last_server_diag));
        }

        const auto rx_begin = SteadyClock::now();
        ret = read(fd_, &net_recv_msg, RECV_BUF_SIZE);
        const auto rx_end = SteadyClock::now();
        monitor.max_rx_us = std::max(monitor.max_rx_us, to_us(rx_end - rx_begin));

        if (ret == RECV_BUF_SIZE)
        {
            ++monitor.rx_ok;
            monitor.consecutive_rx_failures = 0;
            motor_msg_ntohl(&net_recv_msg, &host_recv_msg);
            recv_actual(&host_recv_msg);

            static auto *trace = single_shot_probe::map_trace();
            if (trace != nullptr && single_shot_probe::ready(trace) &&
                trace->ec_send_ns != 0U && trace->ipc_feedback_rx_ns == 0U)
            {
                trace->ipc_feedback_rx_ns = single_shot_probe::now_ns();
            }

            check_startup_zero_offset(monitor.reconnects, true, host_recv_msg.ec_diag);

            const bool feedback_changed =
                !monitor.has_prev_feedback || feedback_packet_changed(host_recv_msg, monitor.prev_feedback);
            if (command_changed && !feedback_changed)
            {
                ++monitor.stale_feedback_cycles;
                if (monitor.stale_feedback_cycles == kStaleFeedbackWarnThreshold ||
                    monitor.stale_feedback_cycles % kStaleFeedbackWarnThreshold == 0)
                {
                    ++monitor.stale_feedback_events;
                    comm_logger().enqueue_event(make_event_record(
                        "stale_feedback",
                        0,
                        0,
                        0,
                        0,
                        monitor.stale_feedback_cycles,
                        monitor.reconnects,
                        true,
                        host_recv_msg.ec_diag));
                }
            }
            else
            {
                monitor.stale_feedback_cycles = 0;
            }

            monitor.prev_cmd = host_send_msg;
            monitor.has_prev_cmd = true;
            monitor.prev_feedback = host_recv_msg;
            monitor.has_prev_feedback = true;
            last_server_diag = host_recv_msg.ec_diag;
            has_server_diag = true;
        }
        else if (ret > 0)
        {
            ++monitor.rx_partial;
            ++monitor.consecutive_rx_failures;
            comm_logger().enqueue_event(make_event_record(
                "rx_partial",
                ret,
                0,
                monitor.consecutive_rx_failures,
                0,
                monitor.stale_feedback_cycles,
                monitor.reconnects,
                has_server_diag,
                last_server_diag));
        }
        else if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            ++monitor.rx_fail;
            ++monitor.consecutive_rx_failures;
            comm_logger().enqueue_event(make_event_record(
                "rx_failed",
                ret,
                errno,
                monitor.consecutive_rx_failures,
                0,
                monitor.stale_feedback_cycles,
                monitor.reconnects,
                has_server_diag,
                last_server_diag));
        }

        const auto cycle_end = SteadyClock::now();
        const auto active_cycle = cycle_end - cycle_begin;
        const uint64_t active_cycle_us = to_us(active_cycle);
        monitor.max_active_cycle_us = std::max(monitor.max_active_cycle_us, active_cycle_us);
        if (active_cycle > kCycleBudget)
        {
            ++monitor.cycle_overruns;
            if (active_cycle > kSevereOverrunThreshold)
            {
                ++monitor.severe_overruns;
                comm_logger().enqueue_event(make_event_record(
                    "cycle_severe_overrun",
                    0,
                    0,
                    0,
                    active_cycle_us,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
        }

        if (cycle_end - monitor.report_start >= kReportPeriod)
        {
            comm_logger().enqueue_snapshot(make_snapshot_record(monitor, has_server_diag, last_server_diag));
            if (has_server_diag && server_diag_has_issue(last_server_diag))
            {
                comm_logger().enqueue_event(make_event_record(
                    "server_diag_abnormal",
                    0,
                    0,
                    0,
                    active_cycle_us,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    true,
                    last_server_diag));
            }
            reset_period_counters(monitor);
            monitor.report_start = cycle_end;
        }

        sleep_for_cycle_budget();
    }
}

void IPCComm::init_motor_msg(struct Motor_master *msg)
{
    memset(msg, 0, sizeof(struct Motor_master));
}

void IPCComm::motor_msg_htonl(struct Motor_master *host_msg, struct Motor_master *net_msg)
{
    memset(net_msg, 0, sizeof(struct Motor_master));
    for (int m = 0; m < master_count; m++)
    {
        for (int i = 0; i < motor_num; i++)
        {
            net_msg->master_id[m].motor_id[i].kp_cmd = htonl(host_msg->master_id[m].motor_id[i].kp_cmd);
            net_msg->master_id[m].motor_id[i].kd_cmd = htonl(host_msg->master_id[m].motor_id[i].kd_cmd);
            net_msg->master_id[m].motor_id[i].position_cmd = htonl(host_msg->master_id[m].motor_id[i].position_cmd);
            net_msg->master_id[m].motor_id[i].velocity_cmd = htonl(host_msg->master_id[m].motor_id[i].velocity_cmd);
            net_msg->master_id[m].motor_id[i].Tor_cmd = htonl(host_msg->master_id[m].motor_id[i].Tor_cmd);
        }
    }
}

void IPCComm::motor_msg_ntohl(struct Motor_master *net_msg, struct Motor_master *host_msg)
{
    memset(host_msg, 0, sizeof(struct Motor_master));
    for (int m = 0; m < master_count; m++)
    {
        for (int i = 0; i < motor_num; i++)
        {
            host_msg->master_id[m].motor_id[i].position_actual =
                ntohl(net_msg->master_id[m].motor_id[i].position_actual);
            host_msg->master_id[m].motor_id[i].velocity_actual =
                ntohl(net_msg->master_id[m].motor_id[i].velocity_actual);
            host_msg->master_id[m].motor_id[i].current_actual =
                ntohl(net_msg->master_id[m].motor_id[i].current_actual);
        }
    }

    host_msg->ec_diag.version = ntohl(net_msg->ec_diag.version);
    host_msg->ec_diag.update_seq = ntohl(net_msg->ec_diag.update_seq);
    for (int m = 0; m < master_count; ++m)
    {
        host_msg->ec_diag.master_diag[m].wc_state = ntohl(net_msg->ec_diag.master_diag[m].wc_state);
        host_msg->ec_diag.master_diag[m].lost_frame_count = ntohl(net_msg->ec_diag.master_diag[m].lost_frame_count);
        host_msg->ec_diag.master_diag[m].lost_frame_delta = ntohl(net_msg->ec_diag.master_diag[m].lost_frame_delta);
        host_msg->ec_diag.master_diag[m].slaves_responding = ntohl(net_msg->ec_diag.master_diag[m].slaves_responding);
        host_msg->ec_diag.master_diag[m].master_al_state = ntohl(net_msg->ec_diag.master_diag[m].master_al_state);
        host_msg->ec_diag.master_diag[m].latency_flag = ntohl(net_msg->ec_diag.master_diag[m].latency_flag);
        host_msg->ec_diag.master_diag[m].latency_max_us = ntohl(net_msg->ec_diag.master_diag[m].latency_max_us);
        host_msg->ec_diag.master_diag[m].latency_min_us = ntohl(net_msg->ec_diag.master_diag[m].latency_min_us);
        host_msg->ec_diag.master_diag[m].latency_avg_us = ntohl(net_msg->ec_diag.master_diag[m].latency_avg_us);
        host_msg->ec_diag.master_diag[m].cycle_counter = ntohl(net_msg->ec_diag.master_diag[m].cycle_counter);
    }
}

void IPCComm::print_send_cmd(const char *prefix, struct Motor_master *msg)
{
    printf("%s:\n", prefix);
    for (int m = 0; m < master_count; m++)
    {
        printf("  Master%d position_cmd: ", m);
        for (int i = 0; i < motor_num; i++)
        {
            printf("motor%d:%d ", i, msg->master_id[m].motor_id[i].position_cmd);
        }
        printf("...\n");
    }
    printf("--------------------------------------------------\n");
}

void IPCComm::print_recv_actual(const char *prefix, struct Motor_master *)
{
    printf("%s:\n", prefix);
    for (size_t i = 0; i < motor_num; ++i)
    {
        printf("motor%zu: kp=%.2f, kd=%.2f, pos=%.2f, vel=%.2f, effort=%.2f\n",
               i,
               temp_global_Joint_data_pub->kp[i],
               temp_global_Joint_data_pub->kd[i],
               temp_global_Joint_data_pub->position[i],
               temp_global_Joint_data_pub->velocity[i],
               temp_global_Joint_data_pub->effort[i]);
    }
}

void IPCComm::recv_actual(struct Motor_master *msg)
{
    for (int i = 0; i < 5; i++)
    {
        if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
        {
            temp_global_Joint_data_pub->position[Motor_Index_Switch[i + 12]] =
                ti5_pulse_to_rad(msg->master_id[1].motor_id[i].position_actual);
            temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i + 12]] =
                ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[i].velocity_actual);
        }
        else if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
        {
            temp_global_Joint_data_pub->position[Motor_Index_Switch[i + 12]] =
                int_to_float(msg->master_id[1].motor_id[i].position_actual);
            temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i + 12]] =
                int_to_float(msg->master_id[1].motor_id[i].velocity_actual);
            temp_global_Joint_data_pub->effort[Motor_Index_Switch[i + 12]] =
                Ti5_current_to_effort(1, i, msg->master_id[1].motor_id[i].current_actual);
        }
    }

    for (int i = 0; i < 6; i++)
    {
        if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
        {
            temp_global_Joint_data_pub->position[Motor_Index_Switch[i + 17]] =
                ti5_pulse_to_rad(msg->master_id[2].motor_id[i].position_actual);
            temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i + 17]] =
                ti5_pulse_to_angular_velocity(msg->master_id[2].motor_id[i].velocity_actual);
        }
        else if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Force_Pos_Mode)
        {
            temp_global_Joint_data_pub->position[Motor_Index_Switch[i + 17]] =
                int_to_float(msg->master_id[2].motor_id[i].position_actual);
            temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i + 17]] =
                int_to_float(msg->master_id[2].motor_id[i].velocity_actual);
            temp_global_Joint_data_pub->effort[Motor_Index_Switch[i + 17]] =
                Ti5_current_to_effort(2, i, msg->master_id[2].motor_id[i].current_actual);
        }
    }

    for (int i = 0; i < 12; i++)
    {
        temp_global_Joint_data_pub->position[Motor_Index_Switch[i]] =
            encos_pulse_to_rad(msg->master_id[0].motor_id[i].position_actual);
        temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i]] =
            encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[i].velocity_actual);
        temp_global_Joint_data_pub->effort[Motor_Index_Switch[i]] =
            encos_pulse_to_effort(msg->master_id[0].motor_id[i].current_actual, i);
    }

    {
        std::lock_guard<std::mutex> lock(ipc_joint_mutex2);
        global_Joint_data_pub = temp_global_Joint_data_pub;
    }
}

void IPCComm::check_startup_zero_offset(uint64_t reconnects,
                                        bool has_server_diag,
                                        const Ethercat_comm_diag &diag)
{
    if (startup_zero_offset_checked_ || temp_global_Joint_data_pub == nullptr)
    {
        return;
    }

    std::string detail;
    int abnormal_joint_count = 0;
    for (int joint_idx = 0; joint_idx < motor_num; ++joint_idx)
    {
        const double position = temp_global_Joint_data_pub->position[joint_idx];
        if (std::fabs(position) <= kStartupZeroOffsetWarnThreshold)
        {
            continue;
        }

        ++abnormal_joint_count;
        char entry[160];
        std::snprintf(entry,
                      sizeof(entry),
                      "%s=%.6f",
                      joint_names[joint_idx].c_str(),
                      position);
        if (!detail.empty())
        {
            detail += "; ";
        }
        detail += entry;
    }

    startup_zero_offset_checked_ = true;
    if (abnormal_joint_count == 0)
    {
        return;
    }

    std::fprintf(stderr,
                 "[Client][Warn] startup zero offset detected: %d/23 joints, threshold=%.6f: %s\n",
                 abnormal_joint_count,
                 kStartupZeroOffsetWarnThreshold,
                 detail.c_str());

    comm_logger().enqueue_event(make_event_record(
        "startup_zero_offset_detected",
        abnormal_joint_count,
        0,
        0,
        0,
        0,
        reconnects,
        has_server_diag,
        diag,
        detail.c_str()));
}

void IPCComm::send_zero_command()
{
    if (fd_ < 0 || temp_global_Joint_data == nullptr)
    {
        return;
    }

    struct Motor_master host_zero_msg, net_zero_msg;
    init_motor_msg(&host_zero_msg);

    for (int i = 0; i < 12; i++)
    {
        host_zero_msg.master_id[0].motor_id[i].kp_cmd =
            encos_kp_to_pulse(temp_global_Joint_data->kp[Motor_Index_Switch[i]]);
        host_zero_msg.master_id[0].motor_id[i].kd_cmd =
            encos_kd_to_pulse(temp_global_Joint_data->kd[Motor_Index_Switch[i]]);
        host_zero_msg.master_id[0].motor_id[i].position_cmd = 0;
        host_zero_msg.master_id[0].motor_id[i].velocity_cmd = 0;
        host_zero_msg.master_id[0].motor_id[i].Tor_cmd = 0;
    }

    for (int i = 0; i < 5; i++)
    {
        host_zero_msg.master_id[1].motor_id[i].kp_cmd =
            float_to_int(temp_global_Joint_data->kp[Motor_Index_Switch[i + 12]]);
        host_zero_msg.master_id[1].motor_id[i].kd_cmd =
            float_to_int(temp_global_Joint_data->kd[Motor_Index_Switch[i + 12]]);
        host_zero_msg.master_id[1].motor_id[i].position_cmd = 0;
        host_zero_msg.master_id[1].motor_id[i].velocity_cmd = 0;
        host_zero_msg.master_id[1].motor_id[i].Tor_cmd = 0;
    }

    for (int i = 0; i < 6; i++)
    {
        host_zero_msg.master_id[2].motor_id[i].kp_cmd =
            float_to_int(temp_global_Joint_data->kp[Motor_Index_Switch[i + 17]]);
        host_zero_msg.master_id[2].motor_id[i].kd_cmd =
            float_to_int(temp_global_Joint_data->kd[Motor_Index_Switch[i + 17]]);
        host_zero_msg.master_id[2].motor_id[i].position_cmd = 0;
        host_zero_msg.master_id[2].motor_id[i].velocity_cmd = 0;
        host_zero_msg.master_id[2].motor_id[i].Tor_cmd = 0;
    }

    motor_msg_htonl(&host_zero_msg, &net_zero_msg);
    int ret = write(fd_, &net_zero_msg, sizeof(struct Motor_master));

    if (ret == sizeof(struct Motor_master))
    {
        comm_logger().enqueue_event(make_event_record(
            "zero_command_sent", ret, 0, 0, 0, 0, 0, false, Ethercat_comm_diag{}));

        struct Motor_master net_recv_zero, host_recv_zero;
        ret = read(fd_, &net_recv_zero, RECV_BUF_SIZE);
        if (ret == RECV_BUF_SIZE)
        {
            motor_msg_ntohl(&net_recv_zero, &host_recv_zero);
            recv_actual(&host_recv_zero);
        }
    }
    else
    {
        comm_logger().enqueue_event(make_event_record(
            "zero_command_send_failed", ret, errno, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
    }
}

#pragma region conversions

double IPCComm::encos_pulse_to_rad(int32_t position_actual)
{
    return (static_cast<double>(position_actual) / 65536.0) * 25.0 - 12.5;
}

double IPCComm::encos_pulse_to_angular_velocity(int32_t velocity_actual)
{
    return -18.0 + static_cast<double>(velocity_actual) * 36.0 / 4095.0;
}

double IPCComm::encos_pulse_to_effort(int32_t current_actual, int32_t encos_motor_idx)
{
    float kt = 0.0f;
    double range = 0.0;
    const int idx = encos_motor_idx % 6;
    if (idx == 0 || idx == 3)
    {
        kt = 2.5f;
        range = 70.0;
    }
    else if (idx == 1)
    {
        kt = 2.35f;
        range = 70.0;
    }
    else if (idx == 2)
    {
        kt = 2.35f;
        range = 60.0;
    }
    else if (idx == 4 || idx == 5)
    {
        kt = 2.8f;
        range = 30.0;
    }
    return (static_cast<double>(current_actual) * range * 2.0 / 4095.0 - range) * kt;
}

float IPCComm::Ti5_current_to_effort(uint8_t master_id, uint8_t motor_idx, int32_t current_actual)
{
    float kt = 0.0f;
    if (master_id == 1)
    {
        if (motor_idx == 0)
            kt = 0.05f;
        else if (motor_idx == 1)
            kt = 0.089f;
        else if (motor_idx == 2)
            kt = 0.05f;
        else if (motor_idx == 3)
            kt = 0.05f;
        else if (motor_idx == 4)
            kt = 0.089f;
    }
    else if (master_id == 2)
    {
        if (motor_idx == 0)
            kt = 0.089f;
        else if (motor_idx == 1)
            kt = 0.05f;
        else if (motor_idx == 2)
            kt = 0.05f;
        else if (motor_idx == 3)
            kt = 0.089f;
        else if (motor_idx == 4)
            kt = 0.096f;
        else if (motor_idx == 5)
            kt = 0.096f;
    }
    return static_cast<float>(current_actual) * kt;
}

int32_t IPCComm::float_to_int(float data)
{
    return static_cast<int32_t>(data * 10000.0f);
}

float IPCComm::int_to_float(int32_t data)
{
    return static_cast<float>(data) / 10000.0f;
}

int32_t IPCComm::encos_kp_to_pulse(double kp)
{
    return static_cast<int32_t>(kp * 1000.0);
}

int32_t IPCComm::encos_kd_to_pulse(double kd)
{
    return static_cast<int32_t>(kd * 1000.0);
}

int32_t IPCComm::encos_effort_to_pulse(double current_actual)
{
    return static_cast<int32_t>(current_actual * 1000.0);
}

int32_t IPCComm::encos_rad_to_degree(double rad)
{
    const double degree = (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
                              ? rad * (180.0 / 3.1415926)
                              : rad;
    return static_cast<int32_t>(1000.0 * degree);
}

int32_t IPCComm::encos_angular_velocity_to_rad_per_min(double angular_velocity)
{
    const double rpm = (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
                           ? angular_velocity * (30.0 / 3.1415926)
                           : angular_velocity;
    return static_cast<int32_t>(1000.0 * rpm);
}

double IPCComm::ti5_pulse_to_rad(int32_t position_actual)
{
    return static_cast<double>(2 * 3.1415926 * position_actual) / 262144.0;
}

double IPCComm::ti5_pulse_to_angular_velocity(int32_t velocity_actual)
{
    return static_cast<double>(velocity_actual) * 2.0 * 3.1415926 / (65536.0 * 51.0);
}

int32_t IPCComm::ti5_rad_to_pulse(double rad)
{
    return static_cast<int32_t>(rad * 262144.0 / (2.0 * 3.1415926));
}

int32_t IPCComm::ti5_angular_velocity_to_pulse(double angular_velocity)
{
    return static_cast<int32_t>(angular_velocity * 65536.0 * 51.0 / (2.0 * 3.1415926));
}

#pragma endregion