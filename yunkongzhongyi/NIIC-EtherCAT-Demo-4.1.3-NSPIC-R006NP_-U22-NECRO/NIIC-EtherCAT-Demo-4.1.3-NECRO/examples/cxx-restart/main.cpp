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
#include <thread>
#include "ecat/niic_api.hpp"

struct program
{
  axis_data* axis;
  power power_;
  move_demo move_;

  void init() {}

  void operator()() {
    power_.axis = axis;
    power_.enable = true;

    move_.axis = axis;
    move_.execute = power_.status;

    // move_.tmp = axis->tmp;

    power_.on_cycle();
    move_.on_cycle();
  }
};

struct program_input
{
  io_data* io;
  int count = 0;
  void init() {}
  void operator()() {
    count++;
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
  io_data* io;
  int count = 0;
  void init() {}
  void operator()() {
    count++;
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

static int axis_count = 0;
static int op_mode = 0;
static std::vector<std::unique_ptr<axis_data>> axes;
static std::vector<std::unique_ptr<io_data>> io_output;
static std::vector<std::unique_ptr<io_data>> io_input;
static std::vector<std::function<void()>> programs;

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
  tt->break_();
}
void config_callback(ecat::task& task);
void receive_callback(ecat::task& task);

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
    ("fileName,f", boost::program_options::value<std::string>(&file_name), "Set eni xml, only for eni mode.")
    ("cycleTime,c", boost::program_options::value<std::int64_t>(&cycle_time)->default_value(1000000), "Set cycle time(ns), only for esi mode.")
    ("shiftTime,s", boost::program_options::value<std::int64_t>(&shiftTime)->default_value(0), "Set cycle shift time(ns), only for esi mode.")
    ("log,l", boost::program_options::value<int>(&log_level)->default_value(2), "Set log level, (0-6).")
    ("priority,p", boost::program_options::value<int>(&priority)->default_value(90), "Set priority of the realtime task (1 - 99)")
    ("affinity,a", boost::program_options::value<int>(&affinity)->default_value(1), "Set CPU affinity of the realtime task")
    ("interval,i", boost::program_options::value<int>(&interval)->default_value(1), "Detection cycle interval")
    ("op_mode,o", boost::program_options::value<int>(&op_mode)->default_value(8), "Set CPU affinity of the realtime task");

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

#if 1
  int slave_num = -1;
  int last_slave_num = -1;
  int i = 0;
  while (++i) {
    std::this_thread::sleep_for(std::chrono::seconds(interval));
    __RT(printf("view info ################################ current index %d\n", i));

    if (tt != nullptr) {
      int ec;
      __RT(printf("view info current tt [Exist] \n"));

      // ec = ecat::rescan_Slaves(tt);
      // if (ec != 0)
      // {
      //   perror("rescan_Slaves");
      // }

      ec = ecat::get_slaves_responding(tt, slave_num);
      if (ec != 0)
      {
        perror("get_slaves_responding");
      }
      __RT(printf("view info current slave_num :%d\n", slave_num));
    }
    else {
      __RT(printf("view info current tt [null] \n"));
    }

    bool changed = false;
    if (last_slave_num != -1 && last_slave_num != slave_num) {
      changed = true;
    }
    last_slave_num = slave_num;
    if (tt != nullptr && changed) {
      __RT(printf("Trigger break.\n"));
      tt->break_();
      tt->wait();
      tt->release();
      tt = nullptr;
    }
    else if (tt == nullptr) {
      // 创建一个新线程
      // 创建新 task 实例
      std::thread t([&]() {

        __RT(printf("\n\nTrigger new\n"));

        ecat::task task_n(0);
        task_n.priority(priority);
        tt = &task_n;

        cpu_set_t cpus;
        CPU_ZERO(&cpus);

        CPU_SET(affinity, &cpus);
        task_n.cpu_affinity(&cpus, sizeof(cpus));
        task_n.record(false);

        std::string file_name_n;
        if (slave_num == 1) {
          file_name_n = "eni_1.xml";
        }
        else {
          file_name_n = "eni_2.xml";
        }

        //eni模式下直接获取指定的xml文件中的配置
        std::cout << "Use eni xml\n";
        task_n.load_eni(file_name_n, cycle_time);

        //设置config的回调函数，将在task_n.start的时候被到用
        task_n.set_config_callback([&] {
          config_callback(task_n);
          });

        //设置cycle的回调函数，会在task_n.start的时候被调用
      //执行状态机切换以及运动控制

      // if (buffer == nullptr)
      // {
      //   printf("======================\n");
      // }

        task_n.set_receive_callback([&] {
          receive_callback(task_n);
          });
        task_n.record(false);
        task_n.start();
        // task_n.wait();
        });
      t.detach();

    }
  }
  // task.load_eni(file_name, cycle_time);
  // tt->start();
  // tt->wait();
#endif

  // 主线程可以继续执行其他任务
  // task.record(true);
  // task.start();
  // task.wait();
  while (true) {
    sleep(1);
  }
  tt->release();
}

void config_callback(ecat::task& task) {
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

        task.try_register_pdo_entry(axis->pv, slave_pos,
          { static_cast<ecat::pdo_index_type>(0x6081 + index_offset), 0 }); // profile velocity
        task.try_register_pdo_entry(axis->pa, slave_pos,
          { static_cast<ecat::pdo_index_type>(0x6083 + index_offset), 0 }); // profile acceleration
        task.try_register_pdo_entry(axis->pd, slave_pos,
          { static_cast<ecat::pdo_index_type>(0x6084 + index_offset), 0 }); // profile deceleration

        // task.try_register_pdo_entry(axis->digital_input, slave_pos,
        //                             {static_cast<ecat::pdo_index_type>(0x60FD + index_offset), 0}); // digital_input
        // task.try_register_pdo_entry(axis->digital_output, slave_pos,
        //                             {static_cast<ecat::pdo_index_type>(0x60FE + index_offset), 0}); // digital_output

        //添加轴，以及对应的program
        axis->mode = op_mode;
        programs.emplace_back(program{ axis.get() });
        axes.push_back(std::move(axis));
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
        task.try_register_pdo_entry(io->io_address, slave_pos, { 0x7000, 0x01 }, &io->io_bit_pos);
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
        task.try_register_pdo_entry(io->io_address, slave_pos, { 0x6000, 0x01 }, &io->io_bit_pos);
        programs.emplace_back(program_input{ io.get() });
        io_input.push_back(std::move(io));

        auto io1 = std::make_unique<io_data>();
        io1->slave_pos = slave_pos;
        io1->io_id = 2;
        task.try_register_pdo_entry(io1->io_address, slave_pos, { 0x6001, 0x01 }, &io1->io_bit_pos);
        programs.emplace_back(program_input{ io1.get() });
        io_input.push_back(std::move(io1));

        auto io2 = std::make_unique<io_data>();
        io2->slave_pos = slave_pos;
        io2->io_id = 3;
        task.try_register_pdo_entry(io2->io_address, slave_pos, { 0x6002, 0x01 }, &io2->io_bit_pos);
        programs.emplace_back(program_input{ io2.get() });
        io_input.push_back(std::move(io2));

        auto io3 = std::make_unique<io_data>();
        io3->slave_pos = slave_pos;
        io3->io_id = 4;
        task.try_register_pdo_entry(io3->io_address, slave_pos, { 0x6003, 0x01 }, &io3->io_bit_pos);
        programs.emplace_back(program_input{ io3.get() });
        io_input.push_back(std::move(io3));
      }
#endif
    }
  }
  ecat::S2SConfig::use().count();
  // axes.at(0)->tmp = true;
}

void receive_callback(ecat::task& task) {
  for (auto& prog : programs)
  {
    prog();
  }
}
