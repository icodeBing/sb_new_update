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
#include "probe/single_shot_probe.h"

namespace
{
  void record_send_probe(int master_index)
  {
    auto *trace = single_shot_probe::map_trace();
    if (trace == nullptr || !single_shot_probe::ready(trace) || trace->ec_send_ns != 0U)
    {
      return;
    }
    if (trace->target_master_index != master_index)
    {
      return;
    }

    const int target_motor = trace->target_motor_index;
    if (target_motor < 0 || target_motor >= motor_number)
    {
      return;
    }

    const auto &motor = motor_msg.master_id[master_index].motor_id[target_motor];
    if (motor.position_cmd == 0 && motor.velocity_cmd == 0 && motor.Tor_cmd == 0)
    {
      return;
    }

    trace->ec_send_ns = single_shot_probe::now_ns();
    std::printf("[Probe][EtherCAT] send_callback master=%d motor=%d ec_send_ns=%llu position_cmd=%d velocity_cmd=%d torque_cmd=%d\n",
                master_index,
                target_motor,
                static_cast<unsigned long long>(trace->ec_send_ns),
                motor.position_cmd,
                motor.velocity_cmd,
                motor.Tor_cmd);
  }
} // namespace

static void usage(const char *program_name)
{
  printf(
      " example:\n"
      "         %s \n"
      "         %s -h\n"
      "         %s -c 500000\n",
      program_name, program_name, program_name);
}

ecat::task *tt0;
ecat::task *tt1;
ecat::task *tt2;

bool motor_enable = true;

void signal_hander(int sig)
{
  signal(SIGINT, SIG_DFL);
  motor_enable = false;
  printf("\nWaiting stop...\n");
  ecat::RTTools::usleep(1000000);
  tt0->break_();
  tt1->break_();
  tt2->break_();
  printf(">>>>>>>> END.\n");
}

int main(int argc, const char **argv)
{
  signal(SIGINT, signal_hander);
  qiuniu_init();

  if (mlockall(MCL_FUTURE | MCL_CURRENT) == -1)
  {
    perror("failed to lock memory\n");
    return 1;
  }

  // spdlog::set_level(spdlog::level::trace);
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

  opts.add_options()("help,h", "This is EtherCAT Demo program.")("affinity,a", boost::program_options::value<int>(&affinity)->default_value(1), "Set CPU affinity of the realtime task")("priority,p", boost::program_options::value<int>(&priority)->default_value(90), "Set priority of the realtime task (1 - 99)")("interval,i", boost::program_options::value<int>(&interval)->default_value(0), "Set optimize jitter parameter. (suggested: 0-20).")
      // ("cycleTime,c", boost::program_options::value<std::int64_t>(&cycle_time)->default_value(1000000), "Set cycle time(ns), only for esi mode.")
      ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(0), "Set cycle shift time(ns), only for esi mode.")
      // ("fileName,f", boost::program_options::value<std::string>(&file_name), "Set eni xml, only for eni mode.")
      ("c0,c0", boost::program_options::value<std::int64_t>(&cycle_time0)->default_value(1000000), "Set the master 0 cycle time (ns), for ESI mode only.")("c1,c1", boost::program_options::value<std::int64_t>(&cycle_time1)->default_value(1000000), "Set the master 1 cycle time (ns), for ESI mode only.")("c2,c2", boost::program_options::value<std::int64_t>(&cycle_time2)->default_value(1000000), "Set the master 2 cycle time (ns), for ESI mode only.")("f0,f0", boost::program_options::value<std::string>(&file_name0), "Set the master 0 eni XML, for ENI mode only.")("f1,f1", boost::program_options::value<std::string>(&file_name1), "Set the master 1 eni XML, for ENI mode only.")("f2,f2", boost::program_options::value<std::string>(&file_name1), "Set the master 1 eni XML, for ENI mode only.")("log,l", boost::program_options::value<int>(&log_level)->default_value(6), "Set log level, (0-6).");

  try
  {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    vm.notify();
  }
  catch (std::exception &e)
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

  for (int i = 0; i < 12; i++)
  {
    motor_msg.master_id[0].motor_id[i].kp_cmd = 100000.0;
    motor_msg.master_id[0].motor_id[i].kd_cmd = 3000.0;
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

  // 若使用同步模式，需要调用startAsMaster()和startAsSlave()；使用异步模式则不需调用

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

  tt0->set_send_callback([]()
                         { record_send_probe(0); });
  tt1->set_send_callback([]()
                         { record_send_probe(1); });
  tt2->set_send_callback([]()
                         { record_send_probe(2); });

  interface_register_tasks(tt0, tt1, tt2);
  interface_task();

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
}
