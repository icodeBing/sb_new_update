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
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

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
ecat::task* tt3;
ecat::task* tt4;
// uint32_t sync_count = 0;
typedef struct MasterData {
  std::vector<MasterProgram*> masters;
  uint32_t cycle_counter = 0; // 周期计数器
};
  
void* thread_func(void* arg)
{
  MasterData* data = static_cast<MasterData*>(arg);
  while(true)
  {
    usleep(10000);  // 10ms，可根据实际周期调整
    data->cycle_counter++;
    // if(data->cycle_counter%10 == 0)
    // {
    //   // 依照实际需求处理ec数据
    //   __RT(printf("----------10ms----------cycle_count=%d\n", data->cycle_counter));
    // }  
  }
  return nullptr;
}

void* imu_thread_func(void* arg)
{
  // UART获取imu数据
  int fd = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);
  if(fd == -1)
  {
    // 打开串口失败
    __RT(printf("open ttyTHS1 error\n"));
    return nullptr;
  }
  struct termios oldtio = {0};
  struct termios newtio = {0};
  tcgetattr(fd, &oldtio);

  newtio.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = 0; // IGNPAR | ICRNL
  newtio.c_oflag = 0;
  newtio.c_lflag = 0; // ICANON
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 1;
  tcflush(fd, TCIOFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);
  // 设置为非阻塞模式
  fcntl(fd, F_SETFL, O_NONBLOCK);
  char buffer[1024] = {0};
  while(true)
  {
    // 打开设备节点，获取节点的文件描述符  O_RDWR:打开的权限，可读可写  O_NOCTTY:进程不受终端影响  O_NDELAY:非阻塞
    int ret = read(fd, buffer, sizeof(buffer));
    if(ret > 0)
    {
      __RT(printf("Start to read params, ret=%d\n", ret));
      for (size_t i = 0; i < ret; i++)
      {
        printf("%x\n", buffer[i]);
      }
      __RT(printf("Finish!!!\n"));

      // 数据处理解析
      // ...
      
    }
    usleep(1000);  // 1ms，按实际需求调整
    
  }
  close(fd);
  return nullptr;
}

void signal_hander(int sig)
{
  signal(SIGINT, SIG_DFL);
  tt0->break_();
  tt1->break_();
  tt2->break_();
  tt3->break_();
  tt4->break_();
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

  // spdlog::set_level(spdlog::level::trace);

  std::int64_t cycle_time0 = 1000000;
  std::int64_t cycle_time1 = 1000000;
  std::int64_t cycle_time2 = 1000000;
  std::int64_t cycle_time3 = 1000000;
  std::int64_t cycle_time4 = 1000000;

  std::string file_name0;
  std::string file_name1;
  std::string file_name2;
  std::string file_name3;
  std::string file_name4;

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
    // ("cycleTime,c", boost::program_options::value<std::int64_t>(&cycle_time)->default_value(1000000), "Set cycle time(ns), only for esi mode.")
    ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(0), "Set cycle shift time(ns), only for esi mode.")
    // ("fileName,f", boost::program_options::value<std::string>(&file_name), "Set eni xml, only for eni mode.")
    ("c0,c0", boost::program_options::value<std::int64_t>(&cycle_time0)->default_value(1000000), "Set the master 0 cycle time (ns), for ESI mode only.")
    ("c1,c1", boost::program_options::value<std::int64_t>(&cycle_time1)->default_value(1000000), "Set the master 1 cycle time (ns), for ESI mode only.")
    ("c2,c2", boost::program_options::value<std::int64_t>(&cycle_time2)->default_value(1000000), "Set the master 2 cycle time (ns), for ESI mode only.")
    ("c3,c3", boost::program_options::value<std::int64_t>(&cycle_time3)->default_value(1000000), "Set the master 2 cycle time (ns), for ESI mode only.")
    ("c4,c4", boost::program_options::value<std::int64_t>(&cycle_time4)->default_value(1000000), "Set the master 2 cycle time (ns), for ESI mode only.")
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

  __RT(printf(" ---------------- start master0 thread ...... \n"));
  MasterProgram master0(0);
  master0.init(1, priority, interval, cycle_time0, shiftTime, file_name0);

  __RT(printf(" ---------------- start master1 thread ...... \n"));
  MasterProgram master1(1);
  master1.init(2, priority, interval, cycle_time1, shiftTime, file_name1);

  __RT(printf(" ---------------- start master2 thread ...... \n"));
  MasterProgram master2(2);
  master2.init(3, priority, interval, cycle_time2, shiftTime, file_name2);

  __RT(printf(" ---------------- start master3 thread ...... \n"));
  MasterProgram master3(3);
  master3.init(4, priority, interval, cycle_time3, shiftTime, file_name3);
  
  __RT(printf(" ---------------- start master4 thread ...... \n"));
  MasterProgram master4(4);
  master4.init(5, priority, interval, cycle_time4, shiftTime, file_name4);

  //若使用同步模式，需要调用startAsMaster()和startAsSlave()；使用异步模式则不需调用

  master0.startAsMaster();
  if (!master1.startAsSlave(&master0, 1))
  {
    printf(" ---------------- start master1 as slave fail ......\n");
    return -1;
  }
  if (!master2.startAsSlave(&master0, 2))
  {
    printf(" ---------------- start master2 as slave fail ......\n");
    return -1;
  }
  if (!master3.startAsSlave(&master0, 3))
  {
    printf(" ---------------- start master3 as slave fail ......\n");
    return -1;
  }
  if (!master4.startAsSlave(&master0, 4))
  {
    printf(" ---------------- start master4 as slave fail ......\n");
    return -1;
  }

  // 共享数据
  MasterData data;
  data.masters.push_back(&master0);
  data.masters.push_back(&master1);
  data.masters.push_back(&master2);
  data.masters.push_back(&master3);
  data.masters.push_back(&master4);

  #if 1
  pthread_t buffer_thread, imu_thread;
  pthread_attr_t attr_buffer, attr_imu;
  sched_param param_buffer, param_imu;
  cpu_set_t cpu_buffer, cpu_imu;

  // buffer线程
  // 初始化
  pthread_attr_init(&attr_buffer);
  // 设置调度策略继承方式
  pthread_attr_setinheritsched(&attr_buffer, PTHREAD_EXPLICIT_SCHED);
  // 设置调度策略
  pthread_attr_setschedpolicy(&attr_buffer, SCHED_FIFO);
  // 设置优先级
  param_buffer.sched_priority = 80;
  pthread_attr_setschedparam(&attr_buffer, &param_buffer);
  // 设置亲核性
  CPU_ZERO(&cpu_buffer);
  CPU_SET(4, &cpu_buffer);
  pthread_attr_setaffinity_np(&attr_buffer, sizeof(cpu_set_t), &cpu_buffer);
  // 创建线程
  __RT(pthread_create(&buffer_thread, &attr_buffer, thread_func, argv));
  // 设置线程名
  __RT(pthread_setname_np(buffer_thread, "buffer_thread"));

  // imu线程
  pthread_attr_init(&attr_imu);
  pthread_attr_setinheritsched(&attr_imu, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr_imu, SCHED_FIFO);
  param_imu.sched_priority = 80;
  pthread_attr_setschedparam(&attr_imu, &param_imu);
  CPU_ZERO(&cpu_imu);
  CPU_SET(5, &cpu_imu);
  pthread_attr_setaffinity_np(&attr_imu, sizeof(cpu_set_t), &cpu_imu);
  __RT(pthread_create(&imu_thread, &attr_imu, imu_thread_func, argv));
  __RT(pthread_setname_np(imu_thread, "imu_thread"));


  #else
    pthread_t buffer_thread;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    sched_param param;
    param.sched_priority = 80;
    // 设置亲和性
    cpu_set_t cpu;
    CPU_ZERO(&cpu);
    CPU_SET(4, &cpu);
    pthread_setaffinity_np(buffer_thread, sizeof(cpu_set_t), &cpu);
    // 创建线程
    __RT(pthread_create(&buffer_thread, &attr, thread_func, argv));
    // 设置调度策略、优先级
    __RT(pthread_setschedparam(buffer_thread, SCHED_FIFO, &param));
    // 设置线程名
    std::string name="buffer_thread";
    pthread_setname_np(buffer_thread, name.c_str());
  #endif


  tt0 = &(master0.task);
  tt1 = &(master1.task);
  tt2 = &(master2.task);
  tt3 = &(master3.task);
  tt4 = &(master4.task);

  master0.start();
  master1.start();
  master2.start();
  master3.start();
  master4.start();

  // 等待线程结束
  pthread_attr_destroy(&attr_buffer);
  __RT(pthread_join(buffer_thread, nullptr));

  pthread_attr_destroy(&attr_imu);
  __RT(pthread_join(imu_thread, nullptr));
  
  master0.task.record(false);
  master1.task.record(false);
  master2.task.record(false);
  master3.task.record(false);
  master4.task.record(false);
  
  master0.wait();
  master1.wait();
  master2.wait();
  master3.wait();
  master4.wait();
  return 0;
}
