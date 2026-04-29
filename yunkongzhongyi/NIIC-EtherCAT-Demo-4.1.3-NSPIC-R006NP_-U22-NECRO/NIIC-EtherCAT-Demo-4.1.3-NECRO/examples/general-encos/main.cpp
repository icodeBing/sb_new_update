#include "ecat/task.hpp"
#include "power.hpp"
#include <iostream>
#include <qiuniu/init.h>
#include <spdlog/spdlog.h>
#include <vector>
#include <boost/program_options.hpp>
#include <signal.h>
#include "ecat/s2s_func.hpp"
#include "move_bd.hpp"
#include "io.hpp"
#include "io_operation.hpp"

#define NANOSECOND 1000000000

typedef std::vector<io_data> io_datas;

struct program
{
  axis_data* axis;
  power power_;
  MoveBD move_;

  void init() {}

  void operator()() {
    power_.axis = axis;
    power_.enable = axis->power_enable;

    move_.axis = axis;
    move_.execute = power_.status;

    power_.on_cycle();
    move_.on_cycle();
  }
};

struct program_io
{
  IoController* io_con;
  void init() {}
  void operator()() {
    io_con->count_flag++;
    // io_con->on_cycle(io_con->control_enable);
    io_con->on_cycle_encos(io_con->slave_pos);
  }
};


static int axis_count = 0;
static std::vector<std::unique_ptr<axis_data>> axes;
static std::vector<std::function<void()>> programs;
static std::vector<std::unique_ptr<IoController>> io_;

static void usage(const char* program_name)
{
  printf(" example:\n"
    "         %s \n"
    "         %s -h\n"
    "         %s -f eni.xml\n"
    "         %s -c 500000\n",
    program_name, program_name, program_name, program_name);
}

ecat::task* tt;
void signal_hander(int sig)
{
  signal(SIGINT, SIG_DFL);
  // 下使能
  for (auto& ax : axes) {
    ax->power_enable = false;
  }
  for (auto& con : io_) {
    con->control_enable = false;
  }
  printf("\nWaiting stop...\n");
  ecat::RTTools::usleep(2000000);
  tt->break_(); //结束任务循环
  printf(">>>>>>>> END.\n");
}

int main(int argc, const char** argv)
{
  // 注册信号处理函数
  signal(SIGINT, signal_hander);
  signal(SIGQUIT, signal_hander);
  signal(SIGABRT, signal_hander);

  qiuniu_init();

  if (mlockall(MCL_FUTURE | MCL_CURRENT) == -1)
  {
    perror("failed to lock memory\n");
    return 1;
  }

  std::string file_name;
  std::int64_t cycle_time;
  std::int64_t shiftTime;
  boost::program_options::options_description opts("all option");
  boost::program_options::variables_map vm;
  int log_level = 0;
  int priority = 90;
  int affinity = 0;
  int op_mode = 0;
  int move_direction = 0;
  int motor_bit = 0;
  int master_id = 0;

  opts.add_options()
    ("help,h", "This is EtherCAT Demo program.")
    ("fileName,f", boost::program_options::value<std::string>(&file_name), "Set eni xml, only for eni mode.")
    ("cycleTime,c", boost::program_options::value<std::int64_t>(&cycle_time)->default_value(1000000), "Set cycle time(ns), only for esi mode.")
    ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(0), "Set cycle shift time(ns), only for esi mode.")
    ("log,l", boost::program_options::value<int>(&log_level)->default_value(2), "Set log level, (0-6).")
    ("priority,p", boost::program_options::value<int>(&priority)->default_value(90), "Set priority of the realtime task (1 - 99)")
    ("affinity,a", boost::program_options::value<int>(&affinity)->default_value(1), "Set CPU affinity of the realtime task")
    ("direction,d", boost::program_options::value<int>(&move_direction)->default_value(1), "Set direction of axis move (0 or 1)")
    ("motor_bit,b", boost::program_options::value<int>(&motor_bit)->default_value(17), "Set motor bit (e.g. 17, 23, ...)")
    ("op_mode,o", boost::program_options::value<int>(&op_mode)->default_value(8), "Set OP mode of the realtime task")
    ("master_id,m", boost::program_options::value<int>(&master_id)->default_value(0), "Set master id");

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

  if (move_direction > 1 || move_direction < 0)
  {
    std::cout << "Error direction!" << "\n";
    std::cout << opts << "\n";
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
  }
  else
  {
    //esi模式下设置默认的循环周期和同步模式
    std::cout << "Use esi xml\n";
    task.cycle_time(cycle_time, shiftTime);
    task.dc_mode(ecat::dc_mode::not_using_dc);
    // task.dc_mode(ecat::dc_mode::master_follow_slave);
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
          axis->axis_id = axis_count++;

          uint32_t supported_drive_modes = 0;

          try {
            // The object dictionary 6502h is used to display servo modes supported by servo drives.
            supported_drive_modes = task.get_master_ptr()->sdo_upload<uint32_t>
              (slave_pos, { static_cast<uint16_t>(0x6502 + index_offset), 0 }, false);
            // homing method 
            if ((supported_drive_modes >> 0x5) & 1)
              task.sdo_download(slave_pos, { static_cast<ecat::pdo_index_type>(0x6098 + index_offset), 0 }, false, (int8_t)35);
            // mode of operation 部分伺服需要通过SDO配置控制模式     
            task.sdo_download(slave_pos, { static_cast<ecat::pdo_index_type>(0x6060 + index_offset), 0 }, false, (int8_t)op_mode);
          }
          catch (std::exception& e) {
            printf("[Warning] SDO init failed, %s\n", e.what());
          }

          //获取PDO对应的domain中的地址偏移量，可根据实际需要添加
          task.try_register_pdo_entry(axis->error_code, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x603f + index_offset), 0 }); // error code
          task.try_register_pdo_entry(axis->control_word, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6040 + index_offset), 0 }); // control word;
          task.try_register_pdo_entry(axis->status_word, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6041 + index_offset), 0 }); // status word
          task.try_register_pdo_entry(axis->mode_of_operation, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6060 + index_offset), 0 }); // mode of operation
          task.try_register_pdo_entry(axis->mode_of_operation_display, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6061 + index_offset), 0 }); // mode of operation display
          task.try_register_pdo_entry(axis->target_position, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x607a + index_offset), 0 }); // target position
          task.try_register_pdo_entry(axis->position_actual_value, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6064 + index_offset), 0 }); // position actual value
          task.try_register_pdo_entry(axis->target_velocity, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x60ff + index_offset), 0 }); // target velocity
          task.try_register_pdo_entry(axis->velocity_actual_value, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x606c + index_offset), 0 }); // velocity actual value
          task.try_register_pdo_entry(axis->target_torque, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6071 + index_offset), 0 }); // target torque
          task.try_register_pdo_entry(axis->torque_actual_value, slave_pos,
            { static_cast<ecat::pdo_index_type>(0x6077 + index_offset), 0 }); // torque actual value

          //添加轴，以及对应的program
          axis->mode = op_mode;
          axis->power_enable = true;
          axis->freq = NANOSECOND / cycle_time;
          axis->direction = move_direction;
          axis->motorBit = motor_bit;
          programs.emplace_back(program{ axis.get() });
          axes.push_back(std::move(axis));
        }
      }
      else if (profile_no == 401 || profile_no == 5001)
      {
        std::unique_ptr<IoController> io_ins;
        if (profile_no == 5001)
          io_ins = std::make_unique<IO_example_rw>();
        else
          io_ins = std::make_unique<IO_example_ro>();

        std::vector<ecat::pdo_entry_info> rxEntries = task.get_io_rx_config(slave_pos, 0);
        for (auto& pdo : rxEntries) {
          if (pdo.entry_idx.idx == 0)
            continue;
          auto io = std::make_unique<io_data>();
          io->slave_pos = slave_pos;
          io->io_idx = pdo.entry_idx.idx;
          io->io_subIdx = pdo.entry_idx.sub_idx;
          io->data_type = io_ins->tans(pdo.datatype);
          task.try_register_pdo_entry(io->io_address, pdo.bit_len, slave_pos, pdo.entry_idx, &io->io_bit_pos);
          io_ins->rx_.push_back(std::move(io));
        }

        std::vector<ecat::pdo_entry_info> txEntries = task.get_io_tx_config(slave_pos, 0);
        for (auto& pdo : txEntries) {
          if (pdo.entry_idx.idx == 0)
            continue;
          auto io = std::make_unique<io_data>();
          io->slave_pos = slave_pos;
          io->io_idx = pdo.entry_idx.idx;
          io->io_subIdx = pdo.entry_idx.sub_idx;
          io->data_type = io_ins->tans(pdo.datatype);
          task.try_register_pdo_entry(io->io_address, pdo.bit_len, slave_pos, pdo.entry_idx, &io->io_bit_pos);
          io_ins->tx_.push_back(std::move(io)); 
        }

        io_ins->bind_pdo();
        io_ins->control_enable = true;
        io_ins->freq = NANOSECOND / cycle_time;
        programs.emplace_back(program_io{ io_ins.get() });
        io_.push_back(std::move(io_ins));
      }
    }
    });


  task.set_receive_callback([&] {
    for (auto& prog : programs)
    {
      prog();
    }
    });

  task.record(false);
  task.start();
  task.wait();
}
