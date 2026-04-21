#include "ecat/task.hpp"
#include <iostream>
#include <qiuniu/init.h>
#include <spdlog/spdlog.h>
#include <vector>
#include <boost/program_options.hpp>
#include <signal.h>
#include "ecat/s2s_func.hpp"

#define contrlword_shutdown(c) (((c) | 0x6) & ~0x81)
#define contrlword_switch_on(c) (((c) | 0x7) & ~0x88)
#define contrlword_disable_voltage(c) ((c) & ~0x82)
#define contrlword_quick_stop(c) (((c) | 0x2) & ~0x84)
#define contrlword_disable_operation(c) (((c) | 0x7) & ~0x88)
#define contrlword_enable_operation(c) (((c) | 0xF) & ~0x80)
#define contrlword_fault_reset(c) ((c) | 0x80)
#define contrlword_new_set_point(c) ((c) | 0x10)
#define contrlword_new_set_point_imm(c) ((c) | 0x30)

typedef enum
{

  no_ready_to_switch_on = 0,
  /* Low level power(e.g. +/- 15V, 5V) has been applied to the drive.
   *  The drive is being initialized or is running self test.
   *  A brake, if present, has to be applied in this state.
   *  The drive function is disable.
   * */

  switch_on_disable,
  /* Drive initialization is complete.
   * The drive parameters have been set up.
   * Drive parameters may be changed.
   * High voltage may not be applied to the dirve.
   * The drive function is disabled.
   * */

  ready_to_switch_on,
  /* High voltage may be applied to the drive.
   * The drive parameters may be changed.
   * The drive function is disabled.
   * */

  switched_on,
  /* High voltage has been applied to the drive.
   * The power amplifier is ready.
   * The dirve parameters may be change.
   * The drive functuion is disable.
   * */
  operation_enable,
  /* No faults have been detected.
   * The dirve  function is enabled and power is apllied to the motor.
   * The dirve function is enable.
   * */

  quick_stop_active,
  /* The drive paramters may be changed.
   * The quick stop function is being executed.
   * The drive function is enabled and power is applied to the motor.
   * */

  fault_reaction_active,
  /* The parameters may be changed.
   * A fault has occurred in the drive.
   * The quick stop functon is being executed.
   * The drive function is enabled and power is applied to the motor.
   * */

  fault,
  /* The drive parameters may be changed.
   * A fault has occurred in the drive.
   * High voltage switch-on/off depends on the application.
   * The drive function is disabled.
   * */
  none
} axis_state_t;

typedef enum
{
  op_mode_no = 0,
  op_mode_pp = 1,
  op_mode_vl = 2,
  op_mode_pv = 3,
  op_mode_hm = 6,
  op_mode_ip = 7,
  op_mode_csp = 8,
  op_mode_csv = 9,
  op_mode_cst = 10
} mode_of_op_t;

static axis_state_t get_axis_state(uint16_t status_word)
{
  if ((status_word & 0x4F) == 0x40)
    return switch_on_disable;
  if ((status_word & 0x6F) == 0x21)
    return ready_to_switch_on;
  if ((status_word & 0x6F) == 0x23)
    return switched_on;
  if ((status_word & 0x6F) == 0x27)
    return operation_enable;
  if ((status_word & 0x6F) == 0x07)
    return quick_stop_active;
  if ((status_word & 0x4F) == 0xF)
    return fault_reaction_active;
  if ((status_word & 0x4F) == 0x08)
    return fault;
  else
    return no_ready_to_switch_on;
}

int power(ecat::PdoRegInfo* control_word, ecat::PdoRegInfo* status_word, ecat::PdoRegInfo* mode_of_operation)
{
  uint16_t status = *((uint16_t*)status_word->dataPtr);
  axis_state_t s = get_axis_state(status);
  int ret = 0;

  switch (s)
  {
  case (no_ready_to_switch_on):
  case (switch_on_disable):
    *((uint16_t*)control_word->dataPtr) = contrlword_shutdown(0);
    break;
  case (ready_to_switch_on):
    *((uint16_t*)control_word->dataPtr) = contrlword_switch_on(0);
    break;
  case (switched_on):
    *((uint16_t*)control_word->dataPtr) = contrlword_enable_operation(0);
    if (mode_of_operation!=nullptr)
      *((uint8_t*)mode_of_operation->dataPtr) = op_mode_csv;
    break;
  case (operation_enable):
    ret = 1;
    break;
  case (quick_stop_active):
  case (fault_reaction_active):
    break;
  case (fault):
    *((uint16_t*)control_word->dataPtr) = contrlword_fault_reset(0);
    break;
  }

  return ret;

}


static int axis_count = 0;
int cycle=0;
int vel = 0;

void io_write(volatile void *dataPtr, size_t bitlen, int value)
{
  switch(bitlen)
  {
    case 1:
      *(uint8_t *)dataPtr = value;
      break;
    case 8:
      *(uint8_t *)dataPtr = value;
      break;
    case 16:
      *(uint16_t *)dataPtr = value;
      break;
    case 32:
      *(uint32_t *)dataPtr = value;
      break;
    default:
      __RT(printf("Invalid bit length: %d\n", bitlen));
  }
}

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
      int n_axis_in_slave = 1, slot_pos;
      int slots_count = task.slots_count(slave_pos);
      // int slots_index_increment = task.slot_index_increment(slave_pos);
      if (slots_count > 0)
      {
        // assert(slots_index_increment != -1);
        n_axis_in_slave = slots_count;
      }
      assert(!(slots_count < 0));
      //CiA402协议模式
      if (profile_no == 402)
      {
        for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos)
        {
          auto rx_pdo=task.get_slot_rx_config(slave_pos, slot_pos);
          auto tx_pdo=task.get_slot_tx_config(slave_pos, slot_pos);
          // 自动注册pdo，可通过task.get_pdo_register_info(slave_pos)获取指定从站的pdo注册信息, 也可以通过task.get_pdo_register_info()获取所有slave的注册信息
          task.auto_register_pdo_entry(rx_pdo, slave_pos, ecat::PdoTypeRx);
          task.auto_register_pdo_entry(tx_pdo, slave_pos, ecat::PdoTypeTx);
        }
      }
      else
      {
        for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos)
        {
          auto rx_pdo=task.get_io_rx_config(slave_pos, slot_pos);
          auto tx_pdo=task.get_io_tx_config(slave_pos, slot_pos);
          task.auto_register_pdo_entry(rx_pdo, slave_pos, ecat::PdoTypeRx);
          task.auto_register_pdo_entry(tx_pdo, slave_pos, ecat::PdoTypeTx);
        }
      }
    }
    ecat::S2SConfig::use().count();
  });
  
  //设置cycle的回调函数，会在task.start的时候被调用
  //执行状态机切换以及运动控制
  task.set_receive_callback([&] {
    static int value = 0;
    if(cycle % 5000 == 0)
    {
      value = !value;
    }
    std::uint16_t slave_count = task.slave_count();
    for(int slave_pos = 0; slave_pos < slave_count; slave_pos++ )
    {
      auto profile_no = task.profile_no(slave_pos);
      if (profile_no == 402)
      {
        // 根据slave_pos和pdo_entry获取对应pdo注册信息
        auto control_word=task.get_pdo_by_entry(slave_pos, {static_cast<ecat::pdo_index_type>(0x6040),0});
        auto status_word=task.get_pdo_by_entry(slave_pos, {static_cast<ecat::pdo_index_type>(0x6041),0});
        auto mode_of_operation=task.get_pdo_by_entry(slave_pos, {static_cast<ecat::pdo_index_type>(0x6060),0});
        auto target_vel=task.get_pdo_by_entry(slave_pos, {static_cast<ecat::pdo_index_type>(0x60ff),0});
        bool printf_flag=false;
        if(control_word!=nullptr&&mode_of_operation!=nullptr&&target_vel!=nullptr)
        {
          if(power(control_word, status_word, mode_of_operation))
          {
            if(vel<=131072)
              vel+=100;
            *((int32_t * )target_vel->dataPtr)=vel;
          }
        }
        else {
          if(!printf_flag)
            __RT(printf("receive_cb::pdo is nullptr\n"));
          printf_flag=true;
        }
      }
      else 
      {
        for(auto iter : task.get_pdo_register_info(slave_pos))
        {
          if(iter->pdoType == ecat::PdoTypeRx)
          {
            io_write(iter->dataPtr, iter->bitLen, value);
          }
          if(cycle%1000==0)
          {
            int value = 0;
            switch(iter->bitLen)
            {
              case 1:
                value = *(int8_t * )iter->dataPtr;
                break;
              case 8:
                value = *(int8_t * )iter->dataPtr;
                break;
              case 16:
                value = *(uint16_t * )iter->dataPtr;
                break;
              case 32:
                value = *(int32_t * )iter->dataPtr;
                break;
              default:
                __RT(printf("Invalid bit length: %d\n", iter->bitLen));
            }
            __RT(printf("receive_cb::slave_pos=%d, {0x%02x, %d}, value=%d\n", slave_pos, iter->entry.idx, iter->entry.sub_idx, value));
          }
        }
      }
    }
    cycle++;
  });
  task.record(false);
  task.start();
  task.wait();
  task.release();
}
