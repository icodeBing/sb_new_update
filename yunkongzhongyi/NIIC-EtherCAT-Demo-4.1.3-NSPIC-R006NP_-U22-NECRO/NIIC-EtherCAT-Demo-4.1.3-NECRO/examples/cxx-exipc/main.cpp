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
#include <necro_ipc.h>
#include "ServoIpcStruct.h"
#include "ec_task.h"

#define EXIPC_PORT_0 0
#define EXIPC_PORT_1 1
#define MAX_AXIS_NUM 64

/********************************ipc ipc***********************************/
static int ipcSocketFd;
static ServoControl_t servoSta[MAX_AXIS_NUM];
static int demo_flag = 0;

static uint8_t goupInfoBuf[sizeof(ServoInfo_t)*MAX_AXIS_NUM + 4 + 4];
static ServoGroupInfo_t *pGroupInfo = ((ServoGroupInfo_t *)(goupInfoBuf + 4));
static IpcEvent_t *pIpcEvtGroupInfo = (IpcEvent_t *)goupInfoBuf;

static uint8_t goupStaBuf[sizeof(ServoSta_t)*MAX_AXIS_NUM + 4 + 4];
static ServoGroupSta_t *pGroupSta = ((ServoGroupSta_t *)(goupStaBuf + 4));
static IpcEvent_t *pIpcEvtGroupSta = (IpcEvent_t *)goupStaBuf;

/****************************************************************************/
struct program
{
  axis_data *axis;
  power power_;
  move_demo move_;

  void init() {}

  void operator()() {
    power_.axis = axis;
    power_.enable = servoSta[axis->axis_id].powerON;

    move_.axis = axis;
    move_.servoControl = servoSta[axis->axis_id];
    move_.servoSta = pGroupSta->sta[axis->axis_id];
    ServoControl_t* al_order = NULL;
    if (!ec_algo_empty())
    {
      al_order = ec_algo_pop();
    }

    if(demo_flag == 1)
    {
      // power_.enable = servoSta[axis->axis_id].powerON;
      move_.execute = power_.status;
      power_.on_cycle();
      move_.on_cycle();
    }
    else 
    {
      if (al_order != NULL)
      {
        if (al_order->axisCount == axis->axis_id)
        {
          servoSta[axis->axis_id].axisCount = al_order->axisCount;
          servoSta[axis->axis_id].powerON = al_order->powerON;
          servoSta[axis->axis_id].targetPosition = al_order->targetPosition;
          servoSta[axis->axis_id].targetVelocity = al_order->targetVelocity;

          ec_algo_ipc_del(al_order);
        }
      }
      if(servoSta[axis->axis_id].powerON != 0)
      {
        power_.on_cycle();
        move_.execute = power_.status;
        move_.qt_on_cycle();
      }
      else
      {
        move_.execute = power_.status;
        axis->p_buf = 0;
        *axis->target_position = *axis->position_actual_value;
        *axis->control_word = 0x6;
      }
   
    }
    pGroupSta->sta[axis->axis_id].axisCount = axis->axis_id;
    pGroupSta->sta[axis->axis_id].errorCode = *axis->error_code;
    pGroupSta->sta[axis->axis_id].statusWord = *axis->status_word;
    pGroupSta->sta[axis->axis_id].curMode = *axis->mode_of_operation;
    pGroupSta->sta[axis->axis_id].curPos = *axis->position_actual_value;
    pGroupSta->sta[axis->axis_id].curVel = *axis->velocity_actual_value;
    ec_ipc_send(ipcSocketFd, (uint8_t *)pIpcEvtGroupSta, sizeof(ServoSta_t)*pGroupSta->slotCount+4+4);
  }
};

struct program_input
{
  io_data *io;
  int count = 0;
  void init() {}
  void operator()() {
    count ++;
    if (!(count % 1000))
    {
      // uint8_t val = *io->io_address;
      uint16_t val = (static_cast<int>(*io->io_address) >> io->io_bit_pos) & 0x01;
      // uint8_t val = (*(uint8_t*)io->io_address >> io->io_bit_pos) & 0x01;
      std::cout << "read slave: " << io->slave_pos << " io: " << std::hex << io->io_id << " value is " << val << "\n";
    }
    count %= 10000;
  }
};

struct program_output
{
  io_data *io;
  int count = 0;
  void init() {}
  void operator()() {
    count ++;
    if (count == 10000) {
      // *(uint16_t*)io->io_address |= (1 << io->io_bit_pos);
      *io->io_address = 0xff;
      std::cout << "write slave: " << io->slave_pos << " count: " << count << " io: " << std::hex << io->io_id << " value is " << ((static_cast<int>(*io->io_address) >> io->io_bit_pos) & 0x01) << "\n";
    }
    else if (count == 5000) {
      // *(uint16_t*)io->io_address |= (1 << io->io_bit_pos);
      *io->io_address = 0x00;
      std::cout << "write slave: " << io->slave_pos << " count: " << count << " io: " << std::hex << io->io_id << " value is " << ((static_cast<int>(*io->io_address) >> io->io_bit_pos) & 0x01) << "\n";
    }
    // std::cout << "write slave: " << io->slave_pos << " io: " << io->io_id << " count: " << count << " value is " << static_cast<int>(*io->io_address)<< "\n";
    count %= 10000;
  }
};

bool recv_flag = true;
static int axis_count = 0;
static std::vector<std::unique_ptr<axis_data>> axes;
static std::vector<std::unique_ptr<io_data>> io_output;
static std::vector<std::unique_ptr<io_data>> io_input;
static std::vector<std::function<void ()>> programs;

static void usage(const char * program_name)
{
  printf(" example:\n"
         "         %s \n"
         "         %s -h\n"
         "         %s -f eni.xml\n"
         "         %s -c 500000\n",
         program_name, program_name, program_name, program_name);
}

ecat::task *tt;
void signal_hander(int sig)
{
  signal(SIGINT, SIG_DFL);
  tt->break_();
}

void asd()
{
  if (!ec_ipc_empty())
  {
    ServoControl_t* qt_order = ec_ipc_pop();
    if (qt_order == NULL)
    {
      return;
    }

    if (qt_order->targetVelocity == 10086)
    {
      demo_flag = qt_order->powerON;
      // delete qt_order;
      ec_algo_ipc_del(qt_order);
      return;
    }

    int curPos = pGroupSta->sta[qt_order->axisCount].curPos;
    int pos = (qt_order->targetPosition - curPos) / 2;

    for (int i = 0; i < 2; i++)
    {
      ServoControl_t tmp;
      tmp.targetPosition = curPos + pos*(i+1);
      if (i == 1)
        tmp.targetPosition = qt_order->targetPosition;
      tmp.axisCount = qt_order->axisCount;
      tmp.targetVelocity = qt_order->targetVelocity;
      tmp.powerON = qt_order->powerON;
      ec_algo_push(&tmp);   // 放入命令缓冲区，ecat每个周期执行一次命令
    }
    demo_flag = 0;
    // delete qt_order;
    ec_algo_ipc_del(qt_order);
  }
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

  std::string file_name;
  std::int64_t cycle_time;
  std::int64_t shiftTime;
  boost::program_options::options_description opts("all option");
  boost::program_options::variables_map vm;
  int log_level = 0;
  int priority = 90;
  int affinity = 0;
  int op_mode = 0;
  int master_id = 0;

  opts.add_options()
      ("help,h", "This is EtherCAT Demo program.")
      ("fileName,f", boost::program_options::value<std::string>(&file_name), "Set eni xml, only for eni mode.")
      ("cycleTime,c", boost::program_options::value<std::int64_t>(&cycle_time)->default_value(1000000), "Set cycle time(ns), only for esi mode.")
      ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(0), "Set cycle shift time(ns), only for esi mode.")
      ("log,l", boost::program_options::value<int>(&log_level)->default_value(2), "Set log level, (0-6).")
      ("priority,p", boost::program_options::value<int>(&priority)->default_value(90), "Set priority of the realtime task (1 - 99)")
      ("affinity,a", boost::program_options::value<int>(&affinity)->default_value(1), "Set CPU affinity of the realtime task")
      ("op_mode,o", boost::program_options::value<int>(&op_mode)->default_value(8), "Set CPU affinity of the realtime task")
      ("master_id,m", boost::program_options::value<int>(&master_id)->default_value(0), "Set master");
  
  try
  {
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, opts), vm);
    vm.notify();
  }
  catch(std::exception &e)
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

  ecat::task task(master_id);
  task.priority(priority);
  tt = &task;

  cpu_set_t cpus;
  CPU_ZERO(&cpus);

  CPU_SET(affinity, &cpus);
  task.cpu_affinity(&cpus, sizeof(cpus));
  task.record(false);

  if (vm.count("fileName"))
  {
    //eni模式下直接获取指定的xml文件中的配置
    std::cout << "Use eni xml\n";
    task.load_eni(file_name, cycle_time);
    ecat::slots_info* mslots_info=task.get_slots_info();
    for(int i=0;i<mslots_info->size();i++)
    {
      ecat::slot_info slot=mslots_info->at(i);
      __RT(printf("print SlotInfo::slave_pos=%d, slot_pos=%d, slot_type=%s, slot_name=%s\n", slot.slave_id, slot.slot_id, slot.slot_type.c_str(), slot.slot_name.c_str()));
    }
  }
  else
  {
    //esi模式下设置默认的循环周期和同步模式
    std::cout << "Use esi xml\n";
    task.cycle_time(cycle_time, shiftTime);
    task.dc_mode(ecat::dc_mode::master_follow_slave);
  }

  //设置config的回调函数，将在task.start的时候被到用
  task.set_config_callback([&] {
    std::uint16_t slave_count = task.slave_count();
    std::uint16_t slave_pos = 0;
    for (slave_pos = 0; slave_pos < slave_count; slave_pos++)
    {
      //获取slave对应的运行协议，如果还未设置则默认为0
      auto profile_no = task.profile_no(slave_pos);
      //CiA402协议模式
      if (profile_no == 402)
      {
        int n_axis_in_slave = 1, slot_pos;
        int slots_count = task.slots_count(slave_pos);
        int slots_index_increment = task.slot_index_increment(slave_pos);
        if (slots_count > 0)
        {
          assert(slots_index_increment != -1);
          n_axis_in_slave = slots_count;
        }
        assert(!(slots_count < 0));
        for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos)
        {
          const int index_offset = slot_pos * slots_index_increment;
          auto axis = std::make_unique<axis_data>();
          axis->slave_pos = slave_pos;
          axis->axis_id = axis_count;
          pGroupInfo->info[axis_count].axisCount = axis_count;
          // task.ec_axis_name(slave_pos, pGroupInfo->info[axis_count].name);
          std::string tmp;
          task.axis_name(slave_pos, tmp);
          snprintf(pGroupInfo->info[axis_count].name, 19, "%s", tmp.c_str());
          const int16_t control=0x80;
          pGroupInfo->info[axis_count].VID = task.axis_vid(slave_pos);
          pGroupInfo->info[axis_count].PID = task.axis_pid(slave_pos);
          printf("name: %s, vid: %d, pid: %d\n", pGroupInfo->info[axis_count].name, pGroupInfo->info[axis_count].VID, pGroupInfo->info[axis_count].PID);

          task.sdo_download(slave_pos, {static_cast<ecat::pdo_index_type>(0x6040 + index_offset), 0}, false, (uint8_t const *)&control, sizeof(control));

          //获取PDO对应的domain中的地址偏移量，可根据实际需要添加
          task.try_register_pdo_entry(axis->error_code, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x603f + index_offset), 0}); // error code
          task.try_register_pdo_entry(axis->control_word, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6040 + index_offset), 0}); // control word;
          task.try_register_pdo_entry(axis->status_word, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6041 + index_offset), 0}); // status word
          task.try_register_pdo_entry(axis->mode_of_operation, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6060 + index_offset), 0}); // mode of operation
          task.try_register_pdo_entry(axis->mode_of_operation_display, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6061 + index_offset), 0}); // mode of operation display
          task.try_register_pdo_entry(axis->target_position, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x607a + index_offset), 0}); // target position
          task.try_register_pdo_entry(axis->position_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6064 + index_offset), 0}); // position actual value
          task.try_register_pdo_entry(axis->target_velocity, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x60ff + index_offset), 0}); // target velocity value
          task.try_register_pdo_entry(axis->velocity_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x606c + index_offset), 0}); // velocity actual value
      
          // task.try_register_pdo_entry(axis->pv, slave_pos,
          //                             {static_cast<ecat::pdo_index_type>(0x6081 + index_offset), 0}); // profile velocity
          // task.try_register_pdo_entry(axis->pa, slave_pos,
          //                             {static_cast<ecat::pdo_index_type>(0x6083 + index_offset), 0}); // profile acceleration
          // task.try_register_pdo_entry(axis->pd, slave_pos,
          //                             {static_cast<ecat::pdo_index_type>(0x6084 + index_offset), 0}); // profile deceleration

          // task.try_register_pdo_entry(axis->digital_input, slave_pos,
          //                             {static_cast<ecat::pdo_index_type>(0x60FD + index_offset), 0}); // digital_input
          // task.try_register_pdo_entry(axis->digital_output, slave_pos,
          //                             {static_cast<ecat::pdo_index_type>(0x60FE + index_offset), 0}); // digital_output

          //添加轴，以及对应的program
          axis->mode = op_mode;
          programs.emplace_back(program{axis.get()});
          axes.push_back(std::move(axis));
          axis_count++;
      }
      }
      if (profile_no == 5001)
      {
        // __RT(printf("slave_pos is %d\n", slave_pos));
#if 1
        std::vector<ecat::pdo_entry_idx> entries = task.mapped_pdo_entries(slave_pos);

        for (int j = 0; j < entries.size(); j++) {
          if (entries[j].idx == 0x0)
          {
            continue;
          }
          // printf("entries.idx %x ,sub_idx %x \n", entries[j].idx, entries[j].sub_idx);
          if (entries[j].idx >= 0x6000 && entries[j].idx < 0x7000)
          {
            auto io = std::make_unique<io_data>();
            io->slave_pos = slave_pos;
            io->io_id = entries[j].idx;
            task.try_register_pdo_entry(io->io_address, slave_pos, entries[j], &io->io_bit_pos);
            programs.emplace_back(program_input{ io.get() });
            io_input.push_back(std::move(io));
          }
          else
          {
            auto io = std::make_unique<io_data>();
            io->slave_pos = slave_pos;
            io->io_id = entries[j].idx;
            task.try_register_pdo_entry(io->io_address, slave_pos, entries[j], &io->io_bit_pos);
            programs.emplace_back(program_output{ io.get() });
            io_output.push_back(std::move(io));
          }
        }
#else
        if (slave_pos == 1)
        {
          auto io = std::make_unique<io_data>();
          io->slave_pos = slave_pos;
          io->io_id = 1;
          task.try_register_pdo_entry(io->io_address, slave_pos, { 0x7000, 0x01 }, & io->io_bit_pos);
          programs.emplace_back(program_output{ io.get() });
          io_output.push_back(std::move(io));

          auto io1 = std::make_unique<io_data>();
          io1->slave_pos = slave_pos;
          io1->io_id = 2;
          task.try_register_pdo_entry(io1->io_address, slave_pos, { 0x7001, 0x01 }, &io1->io_bit_pos);
          programs.emplace_back(program_output{ io1.get() });
          io_output.push_back(std::move(io1));

          auto io2 = std::make_unique<io_data>();
          io2->slave_pos = slave_pos;
          io2->io_id = 3;
          task.try_register_pdo_entry(io2->io_address, slave_pos, { 0x7002, 0x01 }, &io2->io_bit_pos);
          programs.emplace_back(program_output{ io2.get() });
          io_output.push_back(std::move(io2));

          // auto io3 = std::make_unique<io_data>();
          // io3->slave_pos = slave_pos;
          // io3->io_id = 4;
          // task.try_register_pdo_entry(io3->io_address, slave_pos, { 0x7003, 0x01 }, &io3->io_bit_pos);
          // programs.emplace_back(program_output{ io3.get() });
          // io_output.push_back(std::move(io3));

        }

        if (slave_pos == 2)
        {
          auto io = std::make_unique<io_data>();
          io->slave_pos = slave_pos;
          io->io_id = 1;
          task.try_register_pdo_entry(io->io_address, slave_pos, { 0x6000, 0x01 }, & io->io_bit_pos);
          programs.emplace_back(program_input{ io.get() });
          io_input.push_back(std::move(io));

          auto io1 = std::make_unique<io_data>();
          io1->slave_pos = slave_pos;
          io1->io_id = 2;
          task.try_register_pdo_entry(io1->io_address, slave_pos, { 0x6001, 0x01 }, & io1->io_bit_pos);
          programs.emplace_back(program_input{ io1.get() });
          io_input.push_back(std::move(io1));

          auto io2 = std::make_unique<io_data>();
          io2->slave_pos = slave_pos;
          io2->io_id = 3;
          task.try_register_pdo_entry(io2->io_address, slave_pos, { 0x6002, 0x01 }, & io2->io_bit_pos);
          programs.emplace_back(program_input{ io2.get() });
          io_input.push_back(std::move(io2));

          auto io3 = std::make_unique<io_data>();
          io3->slave_pos = slave_pos;
          io3->io_id = 4;
          task.try_register_pdo_entry(io3->io_address, slave_pos, { 0x6003, 0x01 }, & io3->io_bit_pos);
          programs.emplace_back(program_input{ io3.get() });
          io_input.push_back(std::move(io3));
        }
#endif
      }
    }
    ecat::S2SConfig::use().count();
    // axes.at(0)->tmp = true;

    // 发送伺服信息
    pGroupSta->slotCount = axis_count;
    pGroupInfo->slotCount = axis_count;
    ipcSocketFd = ec_ipc_init(EXIPC_PORT_0);
    pIpcEvtGroupSta->ID = static_cast<IpcEventID_t>((sizeof(ServoSta_t)*pGroupSta->slotCount+4) | SERVO_GOURP_STA);
    pIpcEvtGroupInfo->ID = static_cast<IpcEventID_t>((sizeof(ServoInfo_t)*pGroupInfo->slotCount+4) | SERVO_GROUP_INFO);
    ec_ipc_send(ipcSocketFd, (uint8_t *)pIpcEvtGroupInfo, sizeof(ServoInfo_t)*pGroupInfo->slotCount+4+4);
  });
  
  //设置cycle的回调函数，会在task.start的时候被调用
  //执行状态机切换以及运动控制
  
  // if (buffer == nullptr)
  // {
  //   printf("======================\n");
  // }
  task.set_receive_callback([&] {
    for (auto &prog : programs)
    {
      prog();
    }
    // ecat::RunTimeData* buffer = task.get_runtime_data();
    // if (buffer->flag == 1)
    // {
    //   buffer->flag = 0;
    //   __RT(printf("%d, %d, %d\n", buffer->minDelay, buffer->avgDelay, buffer->maxDelay));
    // }
  });

  
  
  
  task.record(false);
  task.start();
  // 设置算法回调函数
  ec_algo_set_cycle(asd);

  // 开启算法线程
  int err = ec_algo_create_thread(55);
  if (err != 0)
  {
    printf("Failed to start algorithm_create_pthread: %s\n", strerror(-err));
    // ec_task_finalize(task);
    task.wait();
    task.release();
    return 1;
  }

  // 接收QT命令
  int fd = ec_ipc_init(EXIPC_PORT_1);
  while (recv_flag)
  {
    ServoControl_t *pTmpReq;
    if (ec_ipc_poll(fd, &pTmpReq) > 0){
      ec_ipc_push(pTmpReq);   // 放入接收缓冲区
    }
  }
  
  task.wait();
  task.release();
  ec_algo_close_thread();
  ec_ipc_deinit(EXIPC_PORT_0);
  ec_ipc_deinit(EXIPC_PORT_1);
}
