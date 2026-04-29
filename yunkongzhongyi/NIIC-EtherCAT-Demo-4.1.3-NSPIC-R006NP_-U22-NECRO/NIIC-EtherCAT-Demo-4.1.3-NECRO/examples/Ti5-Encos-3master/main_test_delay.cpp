#include "ecat/task.hpp"
#include "power.hpp"
#include "move_demo.hpp"
#include <iostream>
#include <qiuniu/init.h>
#include <spdlog/spdlog.h>
#include <vector>
#include <boost/program_options.hpp>
#include "io.hpp"
#include <signal.h>
#include "ecat/s2s_func.hpp"
#include "master_program.hpp"
#include "interface.hpp"

// --- 新增底层时间与原子操作头文件 ---
#include <atomic>
#include <time.h>

// ==========================================
// 单次触发测量 (Single-shot Profiler)
// ==========================================
// 全局标志位和记录时间戳，确保在RT线程中没有任何锁和阻塞
std::atomic<bool> g_latency_recorded{false};
uint64_t g_t_end_ns = 0;
// ==========================================

static void usage(const char* program_name)
{
  printf(
    " example:\n"
    "         %s \n"
    "         %s -h\n"
    "         %s -c 500000\n",
    program_name, program_name, program_name);
}

ecat::task* tt0; 
ecat::task* tt1;
ecat::task* tt2;

bool motor_enable = true;

void signal_hander(int sig)
{
    signal(SIGINT, SIG_DFL);
    motor_enable = false ;
    printf("\nWaiting stop...\n");
    ecat::RTTools::usleep(1000000);
    tt0->break_();
    tt1->break_();
    tt2->break_();
    printf(">>>>>>>> END.\n");
}

int main(int argc, const char** argv)
{
    signal(SIGINT, signal_hander);
    qiuniu_init();

    if (mlockall(MCL_FUTURE | MCL_CURRENT) == -1)
    {
      perror("failed to lock memory\n");
      return 1;
    }

    std::int64_t cycle_time0;
    std::int64_t cycle_time1;
    std::int64_t cycle_time2;

    std::string file_name0;
    std::string file_name1;
    std::string file_name2;

    std::string file_name;
    std::int64_t cycle_time;
    std::int64_t shiftTime;
    boost::program_options::options_description opts("all option");
    boost::program_options::variables_map vm;
    int log_level = 0;
    int priority = 90;
    int affinity = 0;
    int interval = 0;

    opts.add_options()
        ("help,h", "This is EtherCAT Demo program.")
        ("affinity,a", boost::program_options::value<int>(&affinity)->default_value(1), "Set CPU affinity of the realtime task")
        ("priority,p", boost::program_options::value<int>(&priority)->default_value(90), "Set priority of the realtime task (1 - 99)")
        ("interval,i", boost::program_options::value<int>(&interval)->default_value(0), "Set optimize jitter parameter. (suggested: 0-20).")
        ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(0), "Set cycle shift time(ns), only for esi mode.")
        ("c0,c0", boost::program_options::value<std::int64_t>(&cycle_time0)->default_value(1000000), "Set the master 0 cycle time (ns), for ESI mode only.")
        ("c1,c1", boost::program_options::value<std::int64_t>(&cycle_time1)->default_value(1000000), "Set the master 1 cycle time (ns), for ESI mode only.")
        ("c2,c2", boost::program_options::value<std::int64_t>(&cycle_time2)->default_value(1000000), "Set the master 2 cycle time (ns), for ESI mode only.")
        ("f0,f0", boost::program_options::value<std::string>(&file_name0), "Set the master 0 eni XML, for ENI mode only.")
        ("f1,f1", boost::program_options::value<std::string>(&file_name1), "Set the master 1 eni XML, for ENI mode only.")
        ("f2,f2", boost::program_options::value<std::string>(&file_name2), "Set the master 2 eni XML, for ENI mode only.")
        ("log,l", boost::program_options::value<int>(&log_level)->default_value(6), "Set log level, (0-6).")
        ;

    try
    {
      boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
      vm.notify();
    }
    catch (std::exception& e)
    {
      std::cout << e.what() << "\n";
      std::cout << opts << "\n";
      return 1;
    }

    if (vm.count("help"))
    {
      std::cout << opts << "\n";
      usage(argv[0]);
      return 0;
    }

    spdlog::set_level(static_cast<spdlog::level::level_enum>(log_level));

    // 初始化所有电机指令为 0.0
    for(int i=0;i<23;i++)
    {
        motor_msg.master_id[0].motor_id[i].kp_cmd = 100.0;
        motor_msg.master_id[0].motor_id[i].kd_cmd = 3.0;
        motor_msg.master_id[0].motor_id[i].Tor_cmd = 0.0;
        motor_msg.master_id[0].motor_id[i].position_cmd = 0.0;
        motor_msg.master_id[0].motor_id[i].velocity_cmd = 0.0;
    }

    __RT(printf(" ---------------- start master0 thread ...... \n"));
    MasterProgram master0(0);
    master0.init(1, priority, interval, cycle_time0, shiftTime, file_name0);

    __RT(printf(" ---------------- start master1 thread ...... \n"));
    MasterProgram master1(1);
    master1.init(2, priority, interval, cycle_time1, shiftTime, file_name1);

    __RT(printf(" ---------------- start master2 thread ...... \n"));
    MasterProgram master2(2);
    master2.init(3, priority, interval, cycle_time2, shiftTime, file_name2);

    master0.startAsMaster();
    if (!master1.startAsSlave(&master0))
    {
      printf(" ---------------- start master1 as slave fail ......\n");
      return -1;
    }
    if (!master2.startAsSlave(&master0, 2))
    {
      printf(" ---------------- start master2 as slave fail ......\n");
      return -1;
    }

    tt0 = &(master0.task);
    tt1 = &(master1.task);
    tt2 = &(master2.task);

    interface_task();

    // ==========================================
    // 注入点：单次突变检测时间戳
    // ==========================================
    tt0->set_send_callback([]() {
        static int count = 0;
        // 【极客技巧】：利用 position_cmd 从 0.0 突变为 0.5 来感知 ROS 消息的到达
        // 如果电机指令大于 0.1 且还没有被记录过
        if (motor_msg.master_id[0].motor_id[0].position_cmd != 0 ) {
            // 极速获取发送完成那一瞬间的单调时间
            if(count == 1000){
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
            // printf("motor_msg.master_id[0].motor_id[0].position_cmd:%lf\n",motor_msg.master_id[0].motor_id[0].position_cmd);
            // printf("motor_msg.master_id[0].motor_id[1].position_cmd:%lf\n",motor_msg.master_id[0].motor_id[1].position_cmd);
            // printf("motor_msg.master_id[0].motor_id[2].position_cmd:%lf\n",motor_msg.master_id[0].motor_id[2].position_cmd);
            // printf("motor_msg.master_id[0].motor_id[3].position_cmd:%lf\n",motor_msg.master_id[0].motor_id[3].position_cmd);
            // printf("motor_msg.master_id[0].motor_id[4].position_cmd:%lf\n",motor_msg.master_id[0].motor_id[4].position_cmd);
            // printf("motor_msg.master_id[0].motor_id[5].position_cmd:%lf\n",motor_msg.master_id[0].motor_id[5].position_cmd);
            g_t_end_ns = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
            }
            // 锁死记录器，确保只抓取这第一次
            // g_latency_recorded.store(true, std::memory_order_relaxed);
            count ++;
        }
    });
    __RT(printf(" ---------------- 单次延迟探针已就绪，等待 ROS 指令 ...... \n"));
    // ==========================================

    master0.start();
    master1.start();
    master2.start();

    master0.task.record(false);
    master1.task.record(false);
    master2.task.record(false);

    master0.wait();
    master1.wait();
    master2.wait();

    master0.release();
    master1.release();
    master2.release();

    // ==========================================
    // 结果输出：EtherCAT 停机后统一打印，不破坏实时性
    // ==========================================
    double t_end_ms = g_t_end_ns / 1000000.0;
    double t_end_us = g_t_end_ns / 1000.0;

    printf("\n======================================================\n");
    printf("[EtherCAT Profiler] 成功捕获到来自 ROS 的单次指令!\n");
    printf("[EtherCAT Profiler] T_end = %.3f ms (即 %.1f us)\n", t_end_ms, t_end_us);
    printf("------------------------------------------------------\n");
    printf("👉 请手动计算: Latency = T_end(ms) - T_start(ms)\n");
    printf("======================================================\n");

    return 0;
}