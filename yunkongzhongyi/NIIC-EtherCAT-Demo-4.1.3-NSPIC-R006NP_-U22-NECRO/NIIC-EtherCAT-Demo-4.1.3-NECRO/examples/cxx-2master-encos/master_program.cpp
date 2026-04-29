

#include "master_program.hpp"
#include "io_operation.hpp"

MasterProgram::MasterProgram(int master_index):task(master_index)
{
}

MasterProgram::~MasterProgram()
{

}


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


void MasterProgram::init(int affinity, int priority, int interval, std::int64_t cycle_time, std::int64_t shiftTime, const std::string& fileName)
{
  task.priority(priority);

  cpu_set_t cpus;
  CPU_ZERO(&cpus);

  CPU_SET(affinity, &cpus);
  task.cpu_affinity(&cpus, sizeof(cpus));
  task.set_interval(interval);
  run_period = cycle_time;

  if (!fileName.empty())
  {
    //eni模式下直接获取指定的xml文件中的配置
    std::cout << "Use eni xml\n";
    task.load_eni(fileName, cycle_time);
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
      if (profile_no == 401 || profile_no == 5001) 
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
        // io_ins->freq = NANOSECOND / cycle_time;
        programs.emplace_back(program_io{ io_ins.get() });
        io_.push_back(std::move(io_ins));
      }
    }
    });
  
  //设置cycle的回调函数，会在task.start的时候被调用
  //执行状态机切换以及运动控制
  task.set_receive_callback([&] {
    for (auto &prog : programs)
    {
      prog();
    }
  });
}

void MasterProgram::start()
{
    task.start();
}


void MasterProgram::startAsMaster()  
{
  task.setAsMaster();
}

bool MasterProgram::startAsSlave(MasterProgram* master)
{
  printf("..................  startAsSlave, period: %d, master period: %d...\n", run_period, master->run_period);
  if(run_period < master->run_period)
  {
    printf("Start ec as slave failed, the period(%d) must >= the master period(%d).\n", run_period, master->run_period);
    return false;
  }

  if(run_period % master->run_period != 0)
  {
    printf("Start ec as slave failed,  period(%d) must be times of master period(%d).\n", run_period, master->run_period);
    return false;
  }

  task.setAsSlave(&master->task, run_period / master->run_period);
  return true;
}

void MasterProgram::wait()
{
    task.wait();
}

