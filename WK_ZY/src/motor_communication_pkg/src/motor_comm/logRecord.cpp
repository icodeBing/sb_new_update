#include "motor_comm/logRecord.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

namespace motor_comm_log
{
namespace
{
using Microseconds = std::chrono::microseconds;

constexpr const char *kDefaultLogDir = "./motor_comm_logs";
constexpr const char *kSnapshotFileName = "comm_snapshot.csv";
constexpr const char *kEventFileName = "comm_event.csv";

uint64_t realtime_ns()
{
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
    {
        return 0;
    }
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL +
           static_cast<uint64_t>(ts.tv_nsec);
}

const char *wc_state_to_string(int32_t wc_state)
{
    switch (wc_state)
    {
    case 0:
        return "zero";
    case 1:
        return "incomplete";
    case 2:
        return "complete";
    default:
        return "unknown";
    }
}

void fill_log_master_diag(LogMasterDiag (&dest)[master_count], const Ethercat_comm_diag *diag)
{
    for (int master_idx = 0; master_idx < master_count; ++master_idx)
    {
        dest[master_idx] = LogMasterDiag{};
    }

    if (diag == nullptr)
    {
        return;
    }

    for (int master_idx = 0; master_idx < master_count; ++master_idx)
    {
        const auto &source = diag->master_diag[master_idx];
        auto &target = dest[master_idx];
        target.wc_state = source.wc_state;
        target.cycle_counter = static_cast<int32_t>(source.cycle_counter);
        target.lost_frame_count = source.lost_frame_count;
        target.lost_frame_delta = source.lost_frame_delta;
        target.slaves_responding = source.slaves_responding;
        target.master_al_state = source.master_al_state;
        target.latency_min_us = source.latency_min_us;
        target.latency_avg_us = source.latency_avg_us;
        target.latency_max_us = source.latency_max_us;
    }
}

bool ensure_directory_exists(const char *path)
{
    struct stat st;
    if (stat(path, &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }

    if (mkdir(path, 0775) == 0)
    {
        return true;
    }

    return errno == EEXIST;
}

bool write_all(int fd, const char *buffer, size_t size)
{
    size_t written = 0;
    while (written < size)
    {
        const ssize_t ret = write(fd, buffer + written, size - written);
        if (ret < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return false;
        }
        written += static_cast<size_t>(ret);
    }
    return true;
}

bool write_header_if_empty(int fd, const char *header)
{
    struct stat st;
    if (fstat(fd, &st) != 0)
    {
        return false;
    }
    if (st.st_size != 0)
    {
        return true;
    }
    return write_all(fd, header, std::strlen(header));
}

void format_wall_time(uint64_t ns_since_epoch, char *buffer, size_t buffer_size)
{
    const time_t sec = static_cast<time_t>(ns_since_epoch / 1000000000ULL);
    const long nsec = static_cast<long>(ns_since_epoch % 1000000000ULL);
    struct tm tm_value;
    if (localtime_r(&sec, &tm_value) == nullptr)
    {
        std::snprintf(buffer, buffer_size, "1970-01-01 00:00:00.%09ld", nsec);
        return;
    }

    std::snprintf(buffer,
                  buffer_size,
                  "%04d-%02d-%02d %02d:%02d:%02d.%09ld",
                  tm_value.tm_year + 1900,
                  tm_value.tm_mon + 1,
                  tm_value.tm_mday,
                  tm_value.tm_hour,
                  tm_value.tm_min,
                  tm_value.tm_sec,
                  nsec);
}
} // namespace

uint64_t to_us(const std::chrono::steady_clock::duration &duration)
{
    return static_cast<uint64_t>(std::chrono::duration_cast<Microseconds>(duration).count());
}

bool command_packet_changed(const Motor_master &current, const Motor_master &previous)
{
    for (int master_idx = 0; master_idx < master_count; ++master_idx)
    {
        for (int motor_idx = 0; motor_idx < motor_num; ++motor_idx)
        {
            const auto &current_motor = current.master_id[master_idx].motor_id[motor_idx];
            const auto &previous_motor = previous.master_id[master_idx].motor_id[motor_idx];
            if (current_motor.kp_cmd != previous_motor.kp_cmd ||
                current_motor.kd_cmd != previous_motor.kd_cmd ||
                current_motor.Tor_cmd != previous_motor.Tor_cmd ||
                current_motor.position_cmd != previous_motor.position_cmd ||
                current_motor.velocity_cmd != previous_motor.velocity_cmd)
            {
                return true;
            }
        }
    }
    return false;
}

bool feedback_packet_changed(const Motor_master &current, const Motor_master &previous)
{
    for (int master_idx = 0; master_idx < master_count; ++master_idx)
    {
        for (int motor_idx = 0; motor_idx < motor_num; ++motor_idx)
        {
            const auto &current_motor = current.master_id[master_idx].motor_id[motor_idx];
            const auto &previous_motor = previous.master_id[master_idx].motor_id[motor_idx];
            if (current_motor.position_actual != previous_motor.position_actual ||
                current_motor.velocity_actual != previous_motor.velocity_actual ||
                current_motor.current_actual != previous_motor.current_actual)
            {
                return true;
            }
        }
    }
    return false;
}

void reset_period_counters(CommMonitor &monitor)
{
    monitor.cycles = 0;
    monitor.tx_ok = 0;
    monitor.tx_partial = 0;
    monitor.tx_fail = 0;
    monitor.rx_ok = 0;
    monitor.rx_partial = 0;
    monitor.rx_fail = 0;
    monitor.reconnects = 0;
    monitor.cycle_overruns = 0;
    monitor.severe_overruns = 0;
    monitor.stale_feedback_events = 0;
    monitor.max_active_cycle_us = 0;
    monitor.max_tx_us = 0;
    monitor.max_rx_us = 0;
}

bool server_diag_has_issue(const Ethercat_comm_diag &diag)
{
    for (int master_idx = 0; master_idx < master_count; ++master_idx)
    {
        const auto &master_diag = diag.master_diag[master_idx];
        if (master_diag.wc_state != 2 ||
            master_diag.lost_frame_delta > 0 ||
            master_diag.slaves_responding <= 0)
        {
            return true;
        }
    }
    return false;
}

SnapshotRecord make_snapshot_record(const CommMonitor &monitor,
                                    bool has_server_diag,
                                    const Ethercat_comm_diag &diag)
{
    SnapshotRecord record;
    record.realtime_ns = realtime_ns();
    record.cycles = monitor.cycles;
    record.tx_ok = monitor.tx_ok;
    record.tx_partial = monitor.tx_partial;
    record.tx_fail = monitor.tx_fail;
    record.rx_ok = monitor.rx_ok;
    record.rx_partial = monitor.rx_partial;
    record.rx_fail = monitor.rx_fail;
    record.reconnects = monitor.reconnects;
    record.cycle_overruns = monitor.cycle_overruns;
    record.severe_overruns = monitor.severe_overruns;
    record.stale_feedback_events = monitor.stale_feedback_events;
    record.max_active_cycle_us = monitor.max_active_cycle_us;
    record.max_tx_us = monitor.max_tx_us;
    record.max_rx_us = monitor.max_rx_us;
    record.stale_feedback_cycles = monitor.stale_feedback_cycles;
    record.consecutive_tx_failures = monitor.consecutive_tx_failures;
    record.consecutive_rx_failures = monitor.consecutive_rx_failures;
    if (has_server_diag)
    {
        record.diag_version = diag.version;
        record.diag_update_seq = diag.update_seq;
        fill_log_master_diag(record.master_diag, &diag);
    }
    else
    {
        fill_log_master_diag(record.master_diag, nullptr);
    }
    return record;
}

EventRecord make_event_record(const char *event_type,
                              int32_t ret_code,
                              int32_t errno_code,
                              int32_t consecutive_failures,
                              uint64_t active_cycle_us,
                              int32_t stale_feedback_cycles,
                              uint64_t reconnects,
                              bool has_server_diag,
                              const Ethercat_comm_diag &diag,
                              const char *detail)
{
    EventRecord record;
    record.realtime_ns = realtime_ns();
    std::snprintf(record.event_type, sizeof(record.event_type), "%s", event_type);
    std::snprintf(record.detail, sizeof(record.detail), "%s", detail != nullptr ? detail : "");
    record.ret_code = ret_code;
    record.errno_code = errno_code;
    record.consecutive_failures = consecutive_failures;
    record.active_cycle_us = active_cycle_us;
    record.stale_feedback_cycles = stale_feedback_cycles;
    record.reconnects = reconnects;
    if (has_server_diag)
    {
        record.diag_version = diag.version;
        record.diag_update_seq = diag.update_seq;
        fill_log_master_diag(record.master_diag, &diag);
    }
    else
    {
        fill_log_master_diag(record.master_diag, nullptr);
    }
    return record;
}

bool AsyncCsvLogger::start()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (started_)
    {
        return true;
    }

    if (!open_files_locked())
    {
        return false;
    }

    stop_requested_ = false;
    worker_ = std::thread(&AsyncCsvLogger::worker_loop, this);
    started_ = true;
    return true;
}

void AsyncCsvLogger::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_)
        {
            close_files_locked();
            queue_.clear();
            return;
        }
        stop_requested_ = true;
    }

    cv_.notify_one();
    if (worker_.joinable())
    {
        worker_.join();
    }

    std::lock_guard<std::mutex> lock(mutex_);
    started_ = false;
    stop_requested_ = false;
    queue_.clear();
    close_files_locked();
}

void AsyncCsvLogger::enqueue_snapshot(const SnapshotRecord &record)
{
    LogRecord log_record;
    log_record.type = LogRecordType::Snapshot;
    log_record.snapshot = record;
    enqueue(log_record);
}

void AsyncCsvLogger::enqueue_event(const EventRecord &record)
{
    LogRecord log_record;
    log_record.type = LogRecordType::Event;
    log_record.event = record;
    enqueue(log_record);
}

void AsyncCsvLogger::enqueue(const LogRecord &record)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!started_)
    {
        return;
    }
    queue_.push_back(record);
    cv_.notify_one();
}

bool AsyncCsvLogger::open_files_locked()
{
    static constexpr const char *kSnapshotHeader =
        "wall_time,realtime_ns,cycles,tx_ok,tx_partial,tx_fail,rx_ok,rx_partial,rx_fail,"
        "reconnects,cycle_overruns,severe_overruns,max_active_cycle_us,max_tx_us,max_rx_us,"
        "stale_feedback_events,stale_feedback_cycles,consecutive_tx_failures,"
        "consecutive_rx_failures,diag_version,diag_update_seq,"
        "master0_wc_state,master0_cycle_counter,master0_lost_frame_count,master0_lost_frame_delta,"
        "master0_slaves_responding,master0_master_al_state,master0_latency_min_us,"
        "master0_latency_avg_us,master0_latency_max_us,"
        "master1_wc_state,master1_cycle_counter,master1_lost_frame_count,master1_lost_frame_delta,"
        "master1_slaves_responding,master1_master_al_state,master1_latency_min_us,"
        "master1_latency_avg_us,master1_latency_max_us,"
        "master2_wc_state,master2_cycle_counter,master2_lost_frame_count,master2_lost_frame_delta,"
        "master2_slaves_responding,master2_master_al_state,master2_latency_min_us,"
        "master2_latency_avg_us,master2_latency_max_us\n";

    static constexpr const char *kEventHeader =
        "wall_time,realtime_ns,event_type,detail,ret_code,errno_code,consecutive_failures,"
        "active_cycle_us,stale_feedback_cycles,reconnects,diag_version,diag_update_seq,"
        "master0_wc_state,master0_lost_frame_delta,master0_slaves_responding,master0_latency_avg_us,"
        "master1_wc_state,master1_lost_frame_delta,master1_slaves_responding,master1_latency_avg_us,"
        "master2_wc_state,master2_lost_frame_delta,master2_slaves_responding,master2_latency_avg_us\n";

    const char *env_dir = std::getenv("MOTOR_COMM_LOG_DIR");
    const std::string log_dir = (env_dir != nullptr && env_dir[0] != '\0') ? env_dir : kDefaultLogDir;
    if (!ensure_directory_exists(log_dir.c_str()))
    {
        return false;
    }

    const std::string snapshot_path = log_dir + "/" + kSnapshotFileName;
    const std::string event_path = log_dir + "/" + kEventFileName;

    snapshot_fd_ = open(snapshot_path.c_str(), O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC, 0644);
    if (snapshot_fd_ < 0)
    {
        close_files_locked();
        return false;
    }

    event_fd_ = open(event_path.c_str(), O_CREAT | O_WRONLY | O_APPEND | O_CLOEXEC, 0644);
    if (event_fd_ < 0)
    {
        close_files_locked();
        return false;
    }

    if (!write_header_if_empty(snapshot_fd_, kSnapshotHeader) ||
        !write_header_if_empty(event_fd_, kEventHeader))
    {
        close_files_locked();
        return false;
    }

    return true;
}

void AsyncCsvLogger::close_files_locked()
{
    if (snapshot_fd_ >= 0)
    {
        close(snapshot_fd_);
        snapshot_fd_ = -1;
    }
    if (event_fd_ >= 0)
    {
        close(event_fd_);
        event_fd_ = -1;
    }
}

void AsyncCsvLogger::worker_loop()
{
    while (true)
    {
        LogRecord record;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this]() { return stop_requested_ || !queue_.empty(); });
            if (queue_.empty())
            {
                if (stop_requested_)
                {
                    break;
                }
                continue;
            }
            record = queue_.front();
            queue_.pop_front();
        }

        if (record.type == LogRecordType::Snapshot)
        {
            write_snapshot(record.snapshot);
        }
        else
        {
            write_event(record.event);
        }
    }
}

void AsyncCsvLogger::write_snapshot(const SnapshotRecord &record)
{
    if (snapshot_fd_ < 0)
    {
        return;
    }

    char wall_time[64];
    char line[2048];
    format_wall_time(record.realtime_ns, wall_time, sizeof(wall_time));
    const int length = std::snprintf(
        line,
        sizeof(line),
        "%s,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%llu,%d,%d,%d,%u,%u,"
        "%s,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%s,%d,%d,%d,%d,%d,%d,%d,%d,"
        "%s,%d,%d,%d,%d,%d,%d,%d,%d\n",
        wall_time,
        static_cast<unsigned long long>(record.realtime_ns),
        static_cast<unsigned long long>(record.cycles),
        static_cast<unsigned long long>(record.tx_ok),
        static_cast<unsigned long long>(record.tx_partial),
        static_cast<unsigned long long>(record.tx_fail),
        static_cast<unsigned long long>(record.rx_ok),
        static_cast<unsigned long long>(record.rx_partial),
        static_cast<unsigned long long>(record.rx_fail),
        static_cast<unsigned long long>(record.reconnects),
        static_cast<unsigned long long>(record.cycle_overruns),
        static_cast<unsigned long long>(record.severe_overruns),
        static_cast<unsigned long long>(record.max_active_cycle_us),
        static_cast<unsigned long long>(record.max_tx_us),
        static_cast<unsigned long long>(record.max_rx_us),
        static_cast<unsigned long long>(record.stale_feedback_events),
        record.stale_feedback_cycles,
        record.consecutive_tx_failures,
        record.consecutive_rx_failures,
        record.diag_version,
        record.diag_update_seq,
        wc_state_to_string(record.master_diag[0].wc_state),
        record.master_diag[0].cycle_counter,
        record.master_diag[0].lost_frame_count,
        record.master_diag[0].lost_frame_delta,
        record.master_diag[0].slaves_responding,
        record.master_diag[0].master_al_state,
        record.master_diag[0].latency_min_us,
        record.master_diag[0].latency_avg_us,
        record.master_diag[0].latency_max_us,
        wc_state_to_string(record.master_diag[1].wc_state),
        record.master_diag[1].cycle_counter,
        record.master_diag[1].lost_frame_count,
        record.master_diag[1].lost_frame_delta,
        record.master_diag[1].slaves_responding,
        record.master_diag[1].master_al_state,
        record.master_diag[1].latency_min_us,
        record.master_diag[1].latency_avg_us,
        record.master_diag[1].latency_max_us,
        wc_state_to_string(record.master_diag[2].wc_state),
        record.master_diag[2].cycle_counter,
        record.master_diag[2].lost_frame_count,
        record.master_diag[2].lost_frame_delta,
        record.master_diag[2].slaves_responding,
        record.master_diag[2].master_al_state,
        record.master_diag[2].latency_min_us,
        record.master_diag[2].latency_avg_us,
        record.master_diag[2].latency_max_us);

    if (length > 0)
    {
        write_all(snapshot_fd_, line, static_cast<size_t>(length));
    }
}

void AsyncCsvLogger::write_event(const EventRecord &record)
{
    if (event_fd_ < 0)
    {
        return;
    }

    char wall_time[64];
    char line[4096];
    format_wall_time(record.realtime_ns, wall_time, sizeof(wall_time));
    const int length = std::snprintf(
        line,
        sizeof(line),
        "%s,%llu,%s,%s,%d,%d,%d,%llu,%d,%llu,%u,%u,%s,%d,%d,%d,%s,%d,%d,%d,%s,%d,%d,%d\n",
        wall_time,
        static_cast<unsigned long long>(record.realtime_ns),
        record.event_type,
        record.detail,
        record.ret_code,
        record.errno_code,
        record.consecutive_failures,
        static_cast<unsigned long long>(record.active_cycle_us),
        record.stale_feedback_cycles,
        static_cast<unsigned long long>(record.reconnects),
        record.diag_version,
        record.diag_update_seq,
        wc_state_to_string(record.master_diag[0].wc_state),
        record.master_diag[0].lost_frame_delta,
        record.master_diag[0].slaves_responding,
        record.master_diag[0].latency_avg_us,
        wc_state_to_string(record.master_diag[1].wc_state),
        record.master_diag[1].lost_frame_delta,
        record.master_diag[1].slaves_responding,
        record.master_diag[1].latency_avg_us,
        wc_state_to_string(record.master_diag[2].wc_state),
        record.master_diag[2].lost_frame_delta,
        record.master_diag[2].slaves_responding,
        record.master_diag[2].latency_avg_us);

    if (length > 0)
    {
        write_all(event_fd_, line, static_cast<size_t>(length));
    }
}

AsyncCsvLogger &comm_logger()
{
    static AsyncCsvLogger logger;
    return logger;
}
} // namespace motor_comm_log
