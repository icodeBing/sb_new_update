

#include "master_program.hpp"

MasterProgram::MasterProgram(int master_index):task(master_index)
{
}

MasterProgram::~MasterProgram()
{

}

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

          //添加轴，以及对应的program
          programs.emplace_back(program{axis.get()});
          axes.push_back(std::move(axis));
        }
      }
      // if (profile_no == 5001)
      // {
      //   {
      //     auto io = std::make_unique<io_data>();
      //     io->slave_pos = slave_pos;
      //     io->io_id = 1;
      //     task.try_register_pdo_entry(io->io_address, slave_pos,{0x7000, 0}, &io->io_bit_pos);
      //     programs.emplace_back(program_input{io.get()});
      //     io_output.push_back(std::move(io));
      //   }

      //   {
      //     auto io = std::make_unique<io_data>();
      //     io->slave_pos = slave_pos;
      //     io->io_id = 1;
      //     task.try_register_pdo_entry(io->io_address, slave_pos,{0x6000, 0}, &io->io_bit_pos);
      //     programs.emplace_back(program_output{io.get()});
      //     io_input.push_back(std::move(io));
      //   }
      // }
    }
    ecat::S2SConfig::use().count();
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

bool MasterProgram::startAsSlave(MasterProgram* master, int num)
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

  task.setAsSlave(&master->task, run_period / master->run_period, num);
  return true;
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

bool MasterProgram::startAsSlave2(MasterProgram* master)
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

  task.setAsSlave2(&master->task, run_period / master->run_period);
  return true;
}

void MasterProgram::wait()
{
    task.wait();
}

