#include "motor_comm/IPCComm.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <string>
#include <thread>
#include <ctime>
#include <sys/stat.h>

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
constexpr const char *kDefaultLogDir = "./motor_comm_logs";
constexpr const char *kSnapshotFileName = "comm_snapshot.csv";
constexpr const char *kEventFileName = "comm_event.csv";

struct CommMonitor
{
    SteadyClock::time_point report_start = SteadyClock::now();
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

uint64_t to_us(const SteadyClock::duration &duration)
{
    return static_cast<uint64_t>(std::chrono::duration_cast<Microseconds>(duration).count());
}

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
                              const Ethercat_comm_diag &diag)
{
    EventRecord record;
    record.realtime_ns = realtime_ns();
    std::snprintf(record.event_type, sizeof(record.event_type), "%s", event_type);
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

class AsyncCsvLogger
{
public:
    bool start()
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

    void stop()
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

    void enqueue_snapshot(const SnapshotRecord &record)
    {
        LogRecord log_record;
        log_record.type = LogRecordType::Snapshot;
        log_record.snapshot = record;
        enqueue(log_record);
    }

    void enqueue_event(const EventRecord &record)
    {
        LogRecord log_record;
        log_record.type = LogRecordType::Event;
        log_record.event = record;
        enqueue(log_record);
    }

private:
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
        "wall_time,realtime_ns,event_type,ret_code,errno_code,consecutive_failures,"
        "active_cycle_us,stale_feedback_cycles,reconnects,diag_version,diag_update_seq,"
        "master0_wc_state,master0_lost_frame_delta,master0_slaves_responding,master0_latency_avg_us,"
        "master1_wc_state,master1_lost_frame_delta,master1_slaves_responding,master1_latency_avg_us,"
        "master2_wc_state,master2_lost_frame_delta,master2_slaves_responding,master2_latency_avg_us\n";

    void enqueue(const LogRecord &record)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!started_)
        {
            return;
        }
        queue_.push_back(record);
        cv_.notify_one();
    }

    bool open_files_locked()
    {
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

    void close_files_locked()
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

    void worker_loop()
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

    void write_snapshot(const SnapshotRecord &record)
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

    void write_event(const EventRecord &record)
    {
        if (event_fd_ < 0)
        {
            return;
        }

        char wall_time[64];
        char line[1024];
        format_wall_time(record.realtime_ns, wall_time, sizeof(wall_time));
        const int length = std::snprintf(
            line,
            sizeof(line),
            "%s,%llu,%s,%d,%d,%d,%llu,%d,%llu,%u,%u,%s,%d,%d,%d,%s,%d,%d,%d,%s,%d,%d,%d\n",
            wall_time,
            static_cast<unsigned long long>(record.realtime_ns),
            record.event_type,
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

    std::mutex mutex_;
    std::condition_variable cv_;
    std::deque<LogRecord> queue_;
    std::thread worker_;
    bool started_ = false;
    bool stop_requested_ = false;
    int snapshot_fd_ = -1;
    int event_fd_ = -1;
};

AsyncCsvLogger &comm_logger()
{
    static AsyncCsvLogger logger;
    return logger;
}
} // namespace

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

    pthread_attr_init(&ap1);                                    // 初始化线程属性对象ap1
    pthread_attr_setinheritsched(&ap1, PTHREAD_EXPLICIT_SCHED); // 设置线程属性，使其显式继承调度策略
    pthread_attr_setschedpolicy(&ap1, SCHED_FIFO);              // 设置线程的调度策略为SCHED_FIFO（先进先出调度）
    sp1.sched_priority = 50;                                    // 设置调度参数中的线程优先级为50
    pthread_attr_setschedparam(&ap1, &sp1);                     // 将调度参数sp1应用到线程属性ap1中
    CPU_ZERO(&cp1);                                             // 清空CPU集合对象cp1
    CPU_SET(4, &cp1);                                           // 将CPU 4添加到CPU集合对象cp1中，表示该线程可以在CPU 4上运行
    pthread_attr_setaffinity_np(&ap1, sizeof(cp1), &cp1);       // 设置线程的CPU亲和性，使线程p1与CPU集合cp1相关联

#ifdef __RT
    __RT(pthread_create(&p1, &ap1, &IPCComm::thread_fun, this)); // 创建一个线程p1，使用线程属性ap1，线程函数为client，传递给线程函数的参数为NULL
    __RT(pthread_setname_np(p1, "motor-client-cmd"));            // 设置线程p1的名称为"motor-client-cmd"
#else
    pthread_create(&p1, &ap1, &IPCComm::thread_fun, this);
    pthread_setname_np(p1, "motor-client-cmd");
#endif
    pthread_attr_destroy(&ap1); // 销毁线程属性对象ap1，释放相关资源，ap1在进行创建完p1后，就不需要了，所以要释放掉
}

void *IPCComm::thread_fun(void *arg)
{
    IPCComm *self = static_cast<IPCComm *>(arg);
    if (self)
    {
        self->run();
    }
    return nullptr;
}

void IPCComm::stop()
{
    // stop the thread
    g_stop.store(true, std::memory_order_relaxed);

    send_zero_command();

    // join the thread
    int ret = pthread_join(p1, nullptr);
    // 关闭文件描述符
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

// thread fun
void IPCComm::run()
{
    char *devname;
    int ret;
    struct timespec ts;
    struct Motor_master host_send_msg, net_send_msg;
    struct Motor_master net_recv_msg, host_recv_msg;
    CommMonitor monitor;
    bool has_server_diag = false;
    Ethercat_comm_diag last_server_diag{};
    int null_joint_data_cycles = 0;

    // 分配设备名
    if (asprintf(&devname, "/dev/rtp%d", EXIPC_PORT_1) < 0)
    {
        comm_logger().enqueue_event(make_event_record(
            "device_name_alloc_failed", -1, errno, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
        return;
    }

    // 打开设备
    fd_ = open(devname, O_RDWR); // 退出时发送清零指令需要阻塞模式，去掉O_NONBLOCK
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

        // 仅对「全局指针拷贝」加锁，读完立即解锁，锁持有时间<1微秒
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
        } // 花括号结束，lock_guard析构，自动解锁joint_mutex

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

            ts.tv_sec = 0;
            ts.tv_nsec = static_cast<long>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(kCycleBudget).count());
            clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
            continue;
        }

        // 初始化发送结构体
        init_motor_msg(&host_send_msg);

#if 0
        //master1 
        //neck
        host_send_msg.master_id[1].motor_id[0].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[0]);
        host_send_msg.master_id[1].motor_id[0].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[0]);

        //arm_L1
        host_send_msg.master_id[1].motor_id[1].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[1]);
        host_send_msg.master_id[1].motor_id[1].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[1]);

        host_send_msg.master_id[1].motor_id[2].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[2]);
        host_send_msg.master_id[1].motor_id[2].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[2]);


        host_send_msg.master_id[1].motor_id[3].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[3]);
        host_send_msg.master_id[1].motor_id[3].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[3]);


        host_send_msg.master_id[1].motor_id[4].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[4]);
        host_send_msg.master_id[1].motor_id[4].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[4]);

        //master2
        //waist
        host_send_msg.master_id[2].motor_id[0].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[5]);
        host_send_msg.master_id[2].motor_id[0].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[5]);

        host_send_msg.master_id[2].motor_id[1].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[6]);
        host_send_msg.master_id[2].motor_id[1].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[6]);

        //arm_H1
        host_send_msg.master_id[2].motor_id[2].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[7]);
        host_send_msg.master_id[2].motor_id[2].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[7]);


        host_send_msg.master_id[2].motor_id[3].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[8]);
        host_send_msg.master_id[2].motor_id[3].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[8]);

        host_send_msg.master_id[2].motor_id[4].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[9]);
        host_send_msg.master_id[2].motor_id[4].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[9]);


        host_send_msg.master_id[2].motor_id[5].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[10]);
        host_send_msg.master_id[2].motor_id[5].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[10]);
#else
        // master1
        for (int i = 0; i < 5; i++)
        {
            host_send_msg.master_id[1].motor_id[i].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[Motor_Index_Switch[i + 12]]);
            host_send_msg.master_id[1].motor_id[i].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[Motor_Index_Switch[i + 12]]);
        }
        // master2
        for (int i = 0; i < 6; i++)
        {
            host_send_msg.master_id[2].motor_id[i].position_cmd = ti5_rad_to_pulse(temp_global_Joint_data->position[Motor_Index_Switch[i + 17]]);
            host_send_msg.master_id[2].motor_id[i].velocity_cmd = ti5_angular_velocity_to_pulse(temp_global_Joint_data->velocity[Motor_Index_Switch[i + 17]]);
        }
#endif
// master0
// leg_L
#if 0
        host_send_msg.master_id[0].motor_id[0].kp_cmd = temp_global_Joint_data->kp[11];
        host_send_msg.master_id[0].motor_id[0].kd_cmd = temp_global_Joint_data->kd[11];
        host_send_msg.master_id[0].motor_id[0].Tor_cmd = temp_global_Joint_data->effort[11];
        host_send_msg.master_id[0].motor_id[0].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[11]);
        host_send_msg.master_id[0].motor_id[0].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[11]);


        host_send_msg.master_id[0].motor_id[1].kp_cmd = temp_global_Joint_data->kp[12];
        host_send_msg.master_id[0].motor_id[1].kd_cmd = temp_global_Joint_data->kd[12];
        host_send_msg.master_id[0].motor_id[1].Tor_cmd = temp_global_Joint_data->effort[12];
        host_send_msg.master_id[0].motor_id[1].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[12]);
        host_send_msg.master_id[0].motor_id[1].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[12]);

        host_send_msg.master_id[0].motor_id[2].kp_cmd = temp_global_Joint_data->kp[13];
        host_send_msg.master_id[0].motor_id[2].kd_cmd = temp_global_Joint_data->kd[13];
        host_send_msg.master_id[0].motor_id[2].Tor_cmd = temp_global_Joint_data->effort[13];
        host_send_msg.master_id[0].motor_id[2].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[13]);
        host_send_msg.master_id[0].motor_id[2].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[13]);

        host_send_msg.master_id[0].motor_id[3].kp_cmd = temp_global_Joint_data->kp[14];
        host_send_msg.master_id[0].motor_id[3].kd_cmd = temp_global_Joint_data->kd[14];
        host_send_msg.master_id[0].motor_id[3].Tor_cmd = temp_global_Joint_data->effort[14];
        host_send_msg.master_id[0].motor_id[3].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[14]);
        host_send_msg.master_id[0].motor_id[3].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[14]);
        

        host_send_msg.master_id[0].motor_id[4].kp_cmd = temp_global_Joint_data->kp[15];
        host_send_msg.master_id[0].motor_id[4].kd_cmd = temp_global_Joint_data->kd[15];
        host_send_msg.master_id[0].motor_id[4].Tor_cmd = temp_global_Joint_data->effort[15];
        host_send_msg.master_id[0].motor_id[4].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[15]);
        host_send_msg.master_id[0].motor_id[4].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[15]);

        host_send_msg.master_id[0].motor_id[5].kp_cmd = temp_global_Joint_data->kp[16];
        host_send_msg.master_id[0].motor_id[5].kd_cmd = temp_global_Joint_data->kd[16];
        host_send_msg.master_id[0].motor_id[5].Tor_cmd = temp_global_Joint_data->effort[16];
        host_send_msg.master_id[0].motor_id[5].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[16]);
        host_send_msg.master_id[0].motor_id[5].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[16]);

        //leg_H
        host_send_msg.master_id[0].motor_id[6].kp_cmd = temp_global_Joint_data->kp[17];
        host_send_msg.master_id[0].motor_id[6].kd_cmd = temp_global_Joint_data->kd[17];
        host_send_msg.master_id[0].motor_id[6].Tor_cmd = temp_global_Joint_data->effort[17];
        host_send_msg.master_id[0].motor_id[6].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[17]);
        host_send_msg.master_id[0].motor_id[6].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[17]);

        host_send_msg.master_id[0].motor_id[7].kp_cmd = temp_global_Joint_data->kp[18];
        host_send_msg.master_id[0].motor_id[7].kd_cmd = temp_global_Joint_data->kd[18];
        host_send_msg.master_id[0].motor_id[7].Tor_cmd = temp_global_Joint_data->effort[18];
        host_send_msg.master_id[0].motor_id[7].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[18]);
        host_send_msg.master_id[0].motor_id[7].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[18]);

        host_send_msg.master_id[0].motor_id[8].kp_cmd = temp_global_Joint_data->kp[19];
        host_send_msg.master_id[0].motor_id[8].kd_cmd = temp_global_Joint_data->kd[19];
        host_send_msg.master_id[0].motor_id[8].Tor_cmd = temp_global_Joint_data->effort[19];
        host_send_msg.master_id[0].motor_id[8].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[19]);
        host_send_msg.master_id[0].motor_id[8].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[19]);

        host_send_msg.master_id[0].motor_id[9].kp_cmd = temp_global_Joint_data->kp[20];
        host_send_msg.master_id[0].motor_id[9].kd_cmd = temp_global_Joint_data->kd[20];
        host_send_msg.master_id[0].motor_id[9].Tor_cmd = temp_global_Joint_data->effort[20];
        host_send_msg.master_id[0].motor_id[9].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[20]);
        host_send_msg.master_id[0].motor_id[9].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[20]);

        host_send_msg.master_id[0].motor_id[10].kp_cmd = temp_global_Joint_data->kp[21];
        host_send_msg.master_id[0].motor_id[10].kd_cmd = temp_global_Joint_data->kd[21];
        host_send_msg.master_id[0].motor_id[10].Tor_cmd = temp_global_Joint_data->effort[21];
        host_send_msg.master_id[0].motor_id[10].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[21]);
        host_send_msg.master_id[0].motor_id[10].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[21]);

        host_send_msg.master_id[0].motor_id[11].kp_cmd = temp_global_Joint_data->kp[22];
        host_send_msg.master_id[0].motor_id[11].kd_cmd = temp_global_Joint_data->kd[22];
        host_send_msg.master_id[0].motor_id[11].Tor_cmd = temp_global_Joint_data->effort[22];
        host_send_msg.master_id[0].motor_id[11].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[22]);
        host_send_msg.master_id[0].motor_id[11].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[22]);
#else
        for (int i = 0; i < 12; i++)
        {
            host_send_msg.master_id[0].motor_id[i].kp_cmd = encos_kp_to_pulse(temp_global_Joint_data->kp[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].kd_cmd = encos_kd_to_pulse(temp_global_Joint_data->kd[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].Tor_cmd = encos_effort_to_pulse(temp_global_Joint_data->effort[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].position_cmd = encos_rad_to_degree(temp_global_Joint_data->position[Motor_Index_Switch[i]]);
            host_send_msg.master_id[0].motor_id[i].velocity_cmd = encos_angular_velocity_to_rad_per_min(temp_global_Joint_data->velocity[Motor_Index_Switch[i]]);
        }
#endif

        // std::cout << "Tor_cmd_before_4:"<< host_send_msg.master_id[0].motor_id[4].Tor_cmd <<std::endl;
        // std::cout << "Tor_cmd_before_5:"<< host_send_msg.master_id[0].motor_id[5].Tor_cmd <<std::endl;

        // 主机序→网络序
        motor_msg_htonl(&host_send_msg, &net_send_msg);

        // std::cout << "Tor_cmd_after:"<< net_send_msg.master_id[0].motor_id[4].Tor_cmd <<std::endl;

        // 发送指令
        const auto tx_begin = SteadyClock::now();
        ret = write(fd_, &net_send_msg, sizeof(struct Motor_master));
        const auto tx_end = SteadyClock::now();
        monitor.max_tx_us = std::max(monitor.max_tx_us, to_us(tx_end - tx_begin));
        if (ret == sizeof(struct Motor_master))
        {
            ++monitor.tx_ok;
            monitor.consecutive_tx_failures = 0;
            // printf("[Client] 指令发送成功！\n");
            // print_send_cmd("[Client] 发送的指令", &host_send_msg);
        }
        else if (ret < 0)
        {
            const int write_errno = errno;
            ++monitor.tx_fail;
            ++monitor.consecutive_tx_failures;
            if (write_errno != EAGAIN && write_errno != EWOULDBLOCK)
            {
                comm_logger().enqueue_event(make_event_record(
                    "tx_write_error",
                    ret,
                    write_errno,
                    monitor.consecutive_tx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
                // 断线重连
                char *reconnect_devname;
                if (asprintf(&reconnect_devname, "/dev/rtp%d", EXIPC_PORT_1) >= 0)
                {
                    close(fd_);
                    fd_ = open(reconnect_devname, O_RDWR); // 同样去掉O_NONBLOCK
                    free(reconnect_devname);
                    ++monitor.reconnects;
                    comm_logger().enqueue_event(make_event_record(
                        fd_ < 0 ? "client_reconnect_failed" : "client_reconnected",
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
            if (monitor.consecutive_tx_failures == kFailureWarnThreshold)
            {
                comm_logger().enqueue_event(make_event_record(
                    "tx_fail_threshold",
                    ret,
                    write_errno,
                    monitor.consecutive_tx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
        }
        else
        {
            ++monitor.tx_partial;
            ++monitor.consecutive_tx_failures;
            comm_logger().enqueue_event(make_event_record(
                "tx_short_write",
                ret,
                0,
                monitor.consecutive_tx_failures,
                0,
                monitor.stale_feedback_cycles,
                monitor.reconnects,
                has_server_diag,
                last_server_diag));
            if (monitor.consecutive_tx_failures == kFailureWarnThreshold)
            {
                comm_logger().enqueue_event(make_event_record(
                    "tx_partial_threshold",
                    ret,
                    0,
                    monitor.consecutive_tx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
        }

        // 接收实际值
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

            const bool command_changed =
                monitor.has_prev_cmd && command_packet_changed(host_send_msg, monitor.prev_cmd);
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

            monitor.prev_feedback = host_recv_msg;
            monitor.has_prev_feedback = true;
            last_server_diag = host_recv_msg.ec_diag;
            has_server_diag = true;

            // print_recv_actual("[Client] 接收的server实际值", &host_recv_msg);
        }
        else if (ret > 0)
        {
            ++monitor.rx_partial;
            ++monitor.consecutive_rx_failures;
            comm_logger().enqueue_event(make_event_record(
                "rx_short_read",
                ret,
                0,
                monitor.consecutive_rx_failures,
                0,
                monitor.stale_feedback_cycles,
                monitor.reconnects,
                has_server_diag,
                last_server_diag));
            if (monitor.consecutive_rx_failures == kFailureWarnThreshold)
            {
                comm_logger().enqueue_event(make_event_record(
                    "rx_partial_threshold",
                    ret,
                    0,
                    monitor.consecutive_rx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
        }
        else if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
        {
            const int read_errno = errno;
            ++monitor.rx_fail;
            ++monitor.consecutive_rx_failures;
            comm_logger().enqueue_event(make_event_record(
                "rx_read_error",
                ret,
                read_errno,
                monitor.consecutive_rx_failures,
                0,
                monitor.stale_feedback_cycles,
                monitor.reconnects,
                has_server_diag,
                last_server_diag));
            if (monitor.consecutive_rx_failures == kFailureWarnThreshold)
            {
                comm_logger().enqueue_event(make_event_record(
                    "rx_fail_threshold",
                    ret,
                    read_errno,
                    monitor.consecutive_rx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
        }
        else if (ret < 0)
        {
            const int read_errno = errno;
            ++monitor.rx_fail;
            ++monitor.consecutive_rx_failures;
            if (monitor.consecutive_rx_failures == kFailureWarnThreshold)
            {
                comm_logger().enqueue_event(make_event_record(
                    "rx_eagain_threshold",
                    ret,
                    read_errno,
                    monitor.consecutive_rx_failures,
                    0,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
        }

        monitor.prev_cmd = host_send_msg;
        monitor.has_prev_cmd = true;

        const auto active_cycle_us = to_us(SteadyClock::now() - cycle_begin);
        monitor.max_active_cycle_us = std::max(monitor.max_active_cycle_us, active_cycle_us);

        if (active_cycle_us > static_cast<uint64_t>(kCycleBudget.count()))
        {
            ++monitor.cycle_overruns;
            if (active_cycle_us >= static_cast<uint64_t>(kSevereOverrunThreshold.count()))
            {
                ++monitor.severe_overruns;
                comm_logger().enqueue_event(make_event_record(
                    "severe_cycle_overrun",
                    0,
                    0,
                    0,
                    active_cycle_us,
                    monitor.stale_feedback_cycles,
                    monitor.reconnects,
                    has_server_diag,
                    last_server_diag));
            }
            ts.tv_sec = 0;
            ts.tv_nsec = 0;
        }
        else
        {
            const auto sleep_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(kCycleBudget).count() -
                static_cast<long long>(active_cycle_us) * 1000LL;
            ts.tv_sec = 0;
            ts.tv_nsec = static_cast<long>(sleep_ns);
        }

        const auto now = SteadyClock::now();
        if (now - monitor.report_start >= kReportPeriod)
        {
            comm_logger().enqueue_snapshot(make_snapshot_record(monitor, has_server_diag, last_server_diag));
            if (has_server_diag)
            {
                if (server_diag_has_issue(last_server_diag))
                {
                    comm_logger().enqueue_event(make_event_record(
                        "server_diag_abnormal",
                        0,
                        0,
                        0,
                        monitor.max_active_cycle_us,
                        monitor.stale_feedback_cycles,
                        monitor.reconnects,
                        true,
                        last_server_diag));
                }
            }
            reset_period_counters(monitor);
            monitor.report_start = now;
        }

        if (ts.tv_nsec > 0)
        {
            clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
        }
    }

    if (monitor.cycles > 0)
    {
        comm_logger().enqueue_snapshot(make_snapshot_record(monitor, has_server_diag, last_server_diag));
    }
    comm_logger().enqueue_event(make_event_record(
        "client_loop_exit",
        0,
        0,
        0,
        0,
        monitor.stale_feedback_cycles,
        monitor.reconnects,
        has_server_diag,
        last_server_diag));
    return;
}

// 初始化函数
void IPCComm::init_motor_msg(struct Motor_master *msg)
{
    memset(msg, 0, sizeof(struct Motor_master));
}

// 字节序转换函数
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
        // std::cout << (uint32_t)(int32_t)(host_msg->master_id[0].motor_id[4].Tor_cmd) <<std::endl;
    }
}
void IPCComm::motor_msg_ntohl(struct Motor_master *net_msg, struct Motor_master *host_msg)
{
    memset(host_msg, 0, sizeof(struct Motor_master));
    for (int m = 0; m < master_count; m++)
    {
        for (int i = 0; i < motor_num; i++)
        {
            host_msg->master_id[m].motor_id[i].position_actual = ntohl(net_msg->master_id[m].motor_id[i].position_actual);
            host_msg->master_id[m].motor_id[i].velocity_actual = ntohl(net_msg->master_id[m].motor_id[i].velocity_actual);
            host_msg->master_id[m].motor_id[i].current_actual = ntohl(net_msg->master_id[m].motor_id[i].current_actual);
            // std::cout << 1111111111111 << "   "<< net_msg->master_id[m].motor_id[i].current_actual << std::endl;
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
        printf("  Master%d位置指令：", m);
        for (int i = 0; i < motor_num; i++)
            printf("电机%d:%d ", i, msg->master_id[m].motor_id[i].position_cmd);
        printf("...\n");

        printf("  Master%d速度指令：", m);
        for (int i = 0; i < motor_num; i++)
            printf("电机%d:%d ", i, msg->master_id[m].motor_id[i].velocity_cmd);
        printf("...\n");

        printf("  Master%dkp/kd：", m);
        for (int i = 0; i < motor_num; i++)
        {
            printf("电机%d:%d kp:%.4f kd:%.4f", i, msg->master_id[m].motor_id[i].velocity_cmd,
                   msg->master_id[m].motor_id[i].kp_cmd, msg->master_id[m].motor_id[i].kd_cmd);
        }
        printf("...\n");
    }
    printf("--------------------------------------------------\n");
}

void IPCComm::print_recv_actual(const char *prefix, struct Motor_master *msg)
{
    printf("%s:\n", prefix);
    for (size_t i = 0; i < motor_num; ++i)
    {
        printf("电机%d: kp= %.2f, kd= %.2f, 位置=%.2f, 速度=%.2f, 力矩=%.2f.....\n",
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

#if 0
    //matser 1:
    //neck
    temp_global_Joint_data_pub->position[0] =  ti5_pulse_to_rad             (msg->master_id[1].motor_id[0].position_actual);
    temp_global_Joint_data_pub->velocity[0] =  ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[0].velocity_actual);
    //arm_L1
    temp_global_Joint_data_pub->position[1] =  ti5_pulse_to_rad             (msg->master_id[1].motor_id[1].position_actual);
    temp_global_Joint_data_pub->velocity[1] =  ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[1].velocity_actual);

    temp_global_Joint_data_pub->position[2] =  ti5_pulse_to_rad             (msg->master_id[1].motor_id[2].position_actual);
    temp_global_Joint_data_pub->velocity[2] =  ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[2].velocity_actual);

    temp_global_Joint_data_pub->position[3] =  ti5_pulse_to_rad             (msg->master_id[1].motor_id[3].position_actual);
    temp_global_Joint_data_pub->velocity[3] =  ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[3].velocity_actual);

    temp_global_Joint_data_pub->position[4] =  ti5_pulse_to_rad             (msg->master_id[1].motor_id[4].position_actual);
    temp_global_Joint_data_pub->velocity[4] =  ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[4].velocity_actual);

    //matser 2:
    //waist
    temp_global_Joint_data_pub->position[5] =  ti5_pulse_to_rad             (msg->master_id[2].motor_id[0].position_actual);
    temp_global_Joint_data_pub->velocity[5] =  ti5_pulse_to_angular_velocity(msg->master_id[2].motor_id[0].velocity_actual);

    temp_global_Joint_data_pub->position[6] =  ti5_pulse_to_rad             (msg->master_id[2].motor_id[1].position_actual);
    temp_global_Joint_data_pub->velocity[6] =  ti5_pulse_to_angular_velocity(msg->master_id[2].motor_id[1].velocity_actual);

    //arm_R1
    temp_global_Joint_data_pub->position[7] = ti5_pulse_to_rad              (msg->master_id[2].motor_id[2].position_actual);
    temp_global_Joint_data_pub->velocity[7] = ti5_pulse_to_angular_velocity (msg->master_id[2].motor_id[2].velocity_actual);

    temp_global_Joint_data_pub->position[8] = ti5_pulse_to_rad              (msg->master_id[2].motor_id[3].position_actual);
    temp_global_Joint_data_pub->velocity[8] = ti5_pulse_to_angular_velocity (msg->master_id[2].motor_id[3].velocity_actual);

    temp_global_Joint_data_pub->position[9] = ti5_pulse_to_rad              (msg->master_id[2].motor_id[4].position_actual);
    temp_global_Joint_data_pub->velocity[9] = ti5_pulse_to_angular_velocity (msg->master_id[2].motor_id[4].velocity_actual);

    temp_global_Joint_data_pub->position[10] = ti5_pulse_to_rad             (msg->master_id[2].motor_id[5].position_actual);
    temp_global_Joint_data_pub->velocity[10] = ti5_pulse_to_angular_velocity(msg->master_id[2].motor_id[5].velocity_actual);
#else
    // master1
    for (int i = 0; i < 5; i++)
    {
        temp_global_Joint_data_pub->position[Motor_Index_Switch[i + 12]] = ti5_pulse_to_rad(msg->master_id[1].motor_id[i].position_actual);
        temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i + 12]] = ti5_pulse_to_angular_velocity(msg->master_id[1].motor_id[i].velocity_actual);
    }
    // master2
    for (int i = 0; i < 6; i++)
    {
        temp_global_Joint_data_pub->position[Motor_Index_Switch[i + 17]] = ti5_pulse_to_rad(msg->master_id[2].motor_id[i].position_actual);
        temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i + 17]] = ti5_pulse_to_angular_velocity(msg->master_id[2].motor_id[i].velocity_actual);
    }
#endif

#if 0
    //master 0:
    //leg_L
    temp_global_Joint_data_pub->position[11] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[0].position_actual);  
    temp_global_Joint_data_pub->velocity[11] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[0].velocity_actual);

    temp_global_Joint_data_pub->position[12] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[1].position_actual);  
    temp_global_Joint_data_pub->velocity[12] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[1].velocity_actual);

    temp_global_Joint_data_pub->position[13] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[2].position_actual);  
    temp_global_Joint_data_pub->velocity[13] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[2].velocity_actual);

    temp_global_Joint_data_pub->position[14] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[3].position_actual);  
    temp_global_Joint_data_pub->velocity[14] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[3].velocity_actual);

    temp_global_Joint_data_pub->position[15] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[4].position_actual);  
    temp_global_Joint_data_pub->velocity[15] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[4].velocity_actual);

    temp_global_Joint_data_pub->position[16] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[5].position_actual);  
    temp_global_Joint_data_pub->velocity[16] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[5].velocity_actual);

    //leg_R
    temp_global_Joint_data_pub->position[17] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[6].position_actual);  
    temp_global_Joint_data_pub->velocity[17] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[6].velocity_actual);

    temp_global_Joint_data_pub->position[18] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[7].position_actual);  
    temp_global_Joint_data_pub->velocity[18] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[7].velocity_actual);

    temp_global_Joint_data_pub->position[19] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[8].position_actual);  
    temp_global_Joint_data_pub->velocity[19] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[8].velocity_actual);

    temp_global_Joint_data_pub->position[20] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[9].position_actual);  
    temp_global_Joint_data_pub->velocity[20] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[9].velocity_actual);

    temp_global_Joint_data_pub->position[21] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[10].position_actual);  
    temp_global_Joint_data_pub->velocity[21] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[10].velocity_actual);

    temp_global_Joint_data_pub->position[22] =  encos_pulse_to_rad             (msg->master_id[0].motor_id[11].position_actual);  
    temp_global_Joint_data_pub->velocity[22] =  encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[11].velocity_actual);
#else
    for (int i = 0; i < 12; i++)
    {
        temp_global_Joint_data_pub->position[Motor_Index_Switch[i]] = encos_pulse_to_rad(msg->master_id[0].motor_id[i].position_actual);
        temp_global_Joint_data_pub->velocity[Motor_Index_Switch[i]] = encos_pulse_to_angular_velocity(msg->master_id[0].motor_id[i].velocity_actual);
        temp_global_Joint_data_pub->effort[Motor_Index_Switch[i]] = encos_pulse_to_effort(msg->master_id[0].motor_id[i].current_actual, i);
        // std::cout << "temp_global_Joint_data_pub effort:"<< msg->master_id[0].motor_id[i].current_actual << "      \n"<< std::endl;
    }

#endif
    // 仅对「全局指针拷贝」加锁，读完立即解锁，锁持有时间<1微秒
    {
        std::lock_guard<std::mutex> lock(ipc_joint_mutex2);
        global_Joint_data_pub = temp_global_Joint_data_pub;
    } // 花括号结束，lock_guard析构，自动解锁joint_mutex
}

void IPCComm::send_zero_command()
{
    if (fd_ < 0)
        return;
    struct Motor_master host_zero_msg, net_zero_msg;
    init_motor_msg(&host_zero_msg);
    // for(int i = 0;i<23;i++)
    // {
    //     printf("send_zero_command电机%d:kp:%.4f \n", i, temp_global_Joint_data->kp[i]);
    //     printf("send_zero_command电机%d:kd:%.4f \n", i, temp_global_Joint_data->kd[i]);
    // }
    // 清零指定电机的指令（与之前设置的电机对应）
    // master 0  英克斯电机，目前使用的是力位混控模式
    host_zero_msg.master_id[0].motor_id[0].kp_cmd = temp_global_Joint_data->kp[0];
    host_zero_msg.master_id[0].motor_id[0].kd_cmd = temp_global_Joint_data->kd[0];
    host_zero_msg.master_id[0].motor_id[0].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[0].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[0].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[1].kp_cmd = temp_global_Joint_data->kp[1];
    host_zero_msg.master_id[0].motor_id[1].kd_cmd = temp_global_Joint_data->kd[1];
    host_zero_msg.master_id[0].motor_id[1].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[1].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[1].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[2].kp_cmd = temp_global_Joint_data->kp[2];
    host_zero_msg.master_id[0].motor_id[2].kd_cmd = temp_global_Joint_data->kd[2];
    host_zero_msg.master_id[0].motor_id[2].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[2].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[2].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[3].kp_cmd = temp_global_Joint_data->kp[3];
    host_zero_msg.master_id[0].motor_id[3].kd_cmd = temp_global_Joint_data->kd[3];
    host_zero_msg.master_id[0].motor_id[3].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[3].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[3].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[4].kp_cmd = temp_global_Joint_data->kp[4];
    host_zero_msg.master_id[0].motor_id[4].kd_cmd = temp_global_Joint_data->kd[4];
    host_zero_msg.master_id[0].motor_id[4].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[4].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[4].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[5].kp_cmd = temp_global_Joint_data->kp[5];
    host_zero_msg.master_id[0].motor_id[5].kd_cmd = temp_global_Joint_data->kd[5];
    host_zero_msg.master_id[0].motor_id[5].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[5].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[5].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[6].kp_cmd = temp_global_Joint_data->kp[6];
    host_zero_msg.master_id[0].motor_id[6].kd_cmd = temp_global_Joint_data->kd[6];
    host_zero_msg.master_id[0].motor_id[6].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[6].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[6].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[7].kp_cmd = temp_global_Joint_data->kp[7];
    host_zero_msg.master_id[0].motor_id[7].kd_cmd = temp_global_Joint_data->kd[7];
    host_zero_msg.master_id[0].motor_id[7].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[7].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[7].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[8].kp_cmd = temp_global_Joint_data->kp[8];
    host_zero_msg.master_id[0].motor_id[8].kd_cmd = temp_global_Joint_data->kd[8];
    host_zero_msg.master_id[0].motor_id[8].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[8].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[8].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[9].kp_cmd = temp_global_Joint_data->kp[9];
    host_zero_msg.master_id[0].motor_id[9].kd_cmd = temp_global_Joint_data->kd[9];
    host_zero_msg.master_id[0].motor_id[9].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[9].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[9].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[10].kp_cmd = temp_global_Joint_data->kp[10];
    host_zero_msg.master_id[0].motor_id[10].kd_cmd = temp_global_Joint_data->kd[10];
    host_zero_msg.master_id[0].motor_id[10].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[10].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[10].Tor_cmd = 0;

    host_zero_msg.master_id[0].motor_id[11].kp_cmd = temp_global_Joint_data->kp[11];
    host_zero_msg.master_id[0].motor_id[11].kd_cmd = temp_global_Joint_data->kd[11];
    host_zero_msg.master_id[0].motor_id[11].position_cmd = 0;
    host_zero_msg.master_id[0].motor_id[11].velocity_cmd = 0;
    host_zero_msg.master_id[0].motor_id[11].Tor_cmd = 0;

    // master 1 钛虎电机，目前使用的是位置模式，只需要赋值位置和速度
    host_zero_msg.master_id[1].motor_id[0].position_cmd = 0;
    host_zero_msg.master_id[1].motor_id[0].velocity_cmd = 0;

    host_zero_msg.master_id[1].motor_id[1].position_cmd = 0;
    host_zero_msg.master_id[1].motor_id[1].velocity_cmd = 0;

    host_zero_msg.master_id[1].motor_id[2].position_cmd = 0;
    host_zero_msg.master_id[1].motor_id[2].velocity_cmd = 0;

    host_zero_msg.master_id[1].motor_id[3].position_cmd = 0;
    host_zero_msg.master_id[1].motor_id[3].velocity_cmd = 0;

    host_zero_msg.master_id[1].motor_id[4].position_cmd = 0;
    host_zero_msg.master_id[1].motor_id[4].velocity_cmd = 0;

    // master 2  钛虎电机，目前使用的是位置模式，只需要赋值位置和速度
    host_zero_msg.master_id[2].motor_id[0].position_cmd = 0;
    host_zero_msg.master_id[2].motor_id[0].velocity_cmd = 0;

    host_zero_msg.master_id[2].motor_id[1].position_cmd = 0;
    host_zero_msg.master_id[2].motor_id[1].velocity_cmd = 0;

    host_zero_msg.master_id[2].motor_id[2].position_cmd = 0;
    host_zero_msg.master_id[2].motor_id[2].velocity_cmd = 0;

    host_zero_msg.master_id[2].motor_id[3].position_cmd = 0;
    host_zero_msg.master_id[2].motor_id[3].velocity_cmd = 0;

    host_zero_msg.master_id[2].motor_id[4].position_cmd = 0;
    host_zero_msg.master_id[2].motor_id[4].velocity_cmd = 0;

    host_zero_msg.master_id[2].motor_id[5].position_cmd = 0;
    host_zero_msg.master_id[2].motor_id[5].velocity_cmd = 0;

    // 转换为网络序并发送
    motor_msg_htonl(&host_zero_msg, &net_zero_msg);

    int ret = write(fd_, &net_zero_msg, sizeof(struct Motor_master));

    if (ret == sizeof(struct Motor_master))
    {
        comm_logger().enqueue_event(make_event_record(
            "zero_command_sent", ret, 0, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
        // print_send_cmd("[Client] 清零指令", &host_zero_msg);

        // 等待服务器反馈清零后的实际值
        struct Motor_master net_recv_zero, host_recv_zero;
        try
        {
            ret = read(fd_, &net_recv_zero, RECV_BUF_SIZE);
            if (ret == RECV_BUF_SIZE)
            {
                motor_msg_ntohl(&net_recv_zero, &host_recv_zero);
                recv_actual(&host_recv_zero);
                // print_recv_actual("[Client] 清零后服务器反馈", &host_recv_zero);
            }
        }
        catch (const std::exception &e)
        {
            (void)e;
            comm_logger().enqueue_event(make_event_record(
                "zero_command_feedback_exception", 0, 0, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
        }
    }
    else
    {
        comm_logger().enqueue_event(make_event_record(
            "zero_command_send_failed", ret, errno, 0, 0, 0, 0, false, Ethercat_comm_diag{}));
    }
}

#pragma region

// 根据英克斯电机返回报文协议，进行数据转换
double IPCComm::encos_pulse_to_rad(int32_t position_actual)
{
    double rad;
    rad = (double(position_actual) / 65536) * 25.0 - 12.5;
    return rad;
}
double IPCComm::encos_pulse_to_angular_velocity(int32_t velocity_actual)
{
    double angular_velocity;
    angular_velocity = double(-18.0 + velocity_actual * 36.0 / 4095);
    return angular_velocity;
}
// 根据英克斯电机返回的电流值，转为实际扭矩值
double IPCComm::encos_pulse_to_effort(int32_t current_actual, int32_t encos_motor_idx)
{
    float kt;
    double range;
    double effort;
    int idx = encos_motor_idx % 6;
    if (idx == 0 || idx == 3)
    {
        kt = 2.5;
        range = 70;
    }
    else if (idx == 1)
    {
        kt = 2.35;
        range = 70;
    }
    else if (idx == 2)
    {
        kt = 2.35;
        range = 60;
    }
    else if (idx == 4 || idx == 5)
    {
        kt = 2.8;
        range = 30;
    }
    effort = (double(current_actual) * range * 2 / 4095 - range) * kt;
    return effort;
}

int32_t IPCComm::encos_kp_to_pulse(double kp)
{
    return kp * 1000;
}

int32_t IPCComm::encos_kd_to_pulse(double kd)
{
    return kd * 1000;
}

int32_t IPCComm::encos_effort_to_pulse(double current_actual)
{
    return current_actual * 1000;
}

// ipc->niic
int32_t IPCComm::encos_rad_to_degree(double rad)
{
    double degree;
    if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
    {
        degree = rad * (180 / 3.1415926);
    }
    else
    {
        degree = rad;
    }
    return 1000 * degree;
}
int32_t IPCComm::encos_angular_velocity_to_rad_per_min(double angular_velocity)
{
    double rpm;
    if (m_Motor_Ctrl_Mode == Motor_Ctrl_Mode_type::Pos_Mode)
    {
        rpm = angular_velocity * (30.0 / 3.1415926);
    }
    else
    {
        rpm = angular_velocity;
    }
    return 1000 * rpm;
}

double IPCComm::ti5_pulse_to_rad(int32_t position_actual)
{
    double rad;
    // rad = double(position_actual)*360 / (262144.0 * 51.0);
    rad = double((2 * 3.1415926 * position_actual) / 262144);
    return rad;
}
double IPCComm::ti5_pulse_to_angular_velocity(int32_t velocity_actual)
{
    double angular_velocity;
    angular_velocity = (double)velocity_actual * 2 * 3.1415926 / (65536 * 51);
    return angular_velocity;
}
int32_t IPCComm::ti5_rad_to_pulse(double rad)
{
    int32_t position;
    // position = int32_t((rad * 262144.0 * 51.0 / 360.0));
    position = int32_t((rad * 262144.0 / (2 * 3.1415926)));
    return position;
}
int32_t IPCComm::ti5_angular_velocity_to_pulse(double angular_velocity)
{
    int32_t velocity;
    velocity = int32_t((angular_velocity * 65536 * 51 / (2 * 3.1415926)));
    return velocity;
}

#pragma endregion
