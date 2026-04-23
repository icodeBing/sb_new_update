#ifndef MOTOR_COMM_LOG_RECORD_H
#define MOTOR_COMM_LOG_RECORD_H

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

#include "global/golbalParams.h"

namespace motor_comm_log
{
struct CommMonitor
{
    std::chrono::steady_clock::time_point report_start = std::chrono::steady_clock::now();
    uint64_t cycles = 0;
    uint64_t tx_ok = 0;
    uint64_t tx_partial = 0;
    uint64_t tx_fail = 0;
    uint64_t rx_ok = 0;
    uint64_t rx_partial = 0;
    uint64_t rx_fail = 0;
    uint64_t reconnects = 0;
    uint64_t cycle_overruns = 0;
    uint64_t severe_overruns = 0;
    uint64_t stale_feedback_events = 0;
    uint64_t max_active_cycle_us = 0;
    uint64_t max_tx_us = 0;
    uint64_t max_rx_us = 0;
    int consecutive_tx_failures = 0;
    int consecutive_rx_failures = 0;
    int stale_feedback_cycles = 0;
    bool has_prev_cmd = false;
    bool has_prev_feedback = false;
    Motor_master prev_cmd{};
    Motor_master prev_feedback{};
};

struct LogMasterDiag
{
    int32_t wc_state = -1;
    int32_t cycle_counter = -1;
    int32_t lost_frame_count = -1;
    int32_t lost_frame_delta = -1;
    int32_t slaves_responding = -1;
    int32_t master_al_state = -1;
    int32_t latency_min_us = -1;
    int32_t latency_avg_us = -1;
    int32_t latency_max_us = -1;
};

struct SnapshotRecord
{
    uint64_t realtime_ns = 0;
    uint64_t cycles = 0;
    uint64_t tx_ok = 0;
    uint64_t tx_partial = 0;
    uint64_t tx_fail = 0;
    uint64_t rx_ok = 0;
    uint64_t rx_partial = 0;
    uint64_t rx_fail = 0;
    uint64_t reconnects = 0;
    uint64_t cycle_overruns = 0;
    uint64_t severe_overruns = 0;
    uint64_t stale_feedback_events = 0;
    uint64_t max_active_cycle_us = 0;
    uint64_t max_tx_us = 0;
    uint64_t max_rx_us = 0;
    int32_t stale_feedback_cycles = 0;
    int32_t consecutive_tx_failures = 0;
    int32_t consecutive_rx_failures = 0;
    uint32_t diag_version = 0;
    uint32_t diag_update_seq = 0;
    LogMasterDiag master_diag[master_count];
};

struct EventRecord
{
    uint64_t realtime_ns = 0;
    char event_type[48]{};
    char detail[1536]{};
    int32_t ret_code = 0;
    int32_t errno_code = 0;
    int32_t consecutive_failures = 0;
    uint64_t active_cycle_us = 0;
    int32_t stale_feedback_cycles = 0;
    uint64_t reconnects = 0;
    uint32_t diag_version = 0;
    uint32_t diag_update_seq = 0;
    LogMasterDiag master_diag[master_count];
};

uint64_t to_us(const std::chrono::steady_clock::duration &duration);
bool command_packet_changed(const Motor_master &current, const Motor_master &previous);
bool feedback_packet_changed(const Motor_master &current, const Motor_master &previous);
void reset_period_counters(CommMonitor &monitor);
bool server_diag_has_issue(const Ethercat_comm_diag &diag);

SnapshotRecord make_snapshot_record(const CommMonitor &monitor,
                                    bool has_server_diag,
                                    const Ethercat_comm_diag &diag);

EventRecord make_event_record(const char *event_type,
                              int32_t ret_code,
                              int32_t errno_code,
                              int32_t consecutive_failures,
                              uint64_t active_cycle_us,
                              int32_t stale_feedback_cycles,
                              uint64_t reconnects,
                              bool has_server_diag,
                              const Ethercat_comm_diag &diag,
                              const char *detail = nullptr);

class AsyncCsvLogger
{
public:
    bool start();
    void stop();
    void enqueue_snapshot(const SnapshotRecord &record);
    void enqueue_event(const EventRecord &record);

private:
    enum class LogRecordType
    {
        Snapshot,
        Event,
    };

    struct LogRecord
    {
        LogRecordType type = LogRecordType::Snapshot;
        SnapshotRecord snapshot{};
        EventRecord event{};
    };

    void enqueue(const LogRecord &record);
    bool open_files_locked();
    void close_files_locked();
    void worker_loop();
    void write_snapshot(const SnapshotRecord &record);
    void write_event(const EventRecord &record);

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<LogRecord> queue_;
    std::thread worker_;
    bool started_ = false;
    bool stop_requested_ = false;
    int snapshot_fd_ = -1;
    int event_fd_ = -1;
};

AsyncCsvLogger &comm_logger();
} // namespace motor_comm_log

#endif
