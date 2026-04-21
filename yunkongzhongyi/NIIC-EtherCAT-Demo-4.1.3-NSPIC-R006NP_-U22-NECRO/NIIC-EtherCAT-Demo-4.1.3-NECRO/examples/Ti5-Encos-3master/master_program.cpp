

#include "master_program.hpp"
#include "io_operation.hpp"
#include "probe/single_shot_probe.h"

#include <unistd.h> // getpid
#include <thread>

#define NANOSECOND 1000000000

// MasterProgram::MasterProgram(int master_index):task(master_index)
// {
//   // std::cout<<"master_id :  "<<master_index<<std::endl;
// }
MasterProgram::MasterProgram(int master_index)
    : task(master_index), master_index_(master_index)
{
  // std::cout<<"master_id :  "<<master_index<<std::endl;
}

MasterProgram::~MasterProgram()
{
}

struct program_io
{
  IoController *io_con;
  void init() {}
  void operator()()
  {

    // auto now = std::chrono::steady_clock::now();
    // std::cout << "IO Slave " << io_con->slave_pos << " cycle at "
    //           << std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count()
    //           << " us" << std::endl;

    io_con->count_flag++;
    // io_con->on_cycle(io_con->control_enable);
    io_con->on_cycle_encos(io_con->slave_pos);
  }
};

static int axis_count = 0;
static std::vector<std::unique_ptr<axis_data>> axes;
static std::vector<std::function<void()>> programs;
static std::vector<std::unique_ptr<IoController>> io_;

namespace
{
void record_receive_probe(int master_index)
{
  auto *trace = single_shot_probe::map_trace();
  if (trace == nullptr || !single_shot_probe::ready(trace) ||
      trace->ec_send_ns == 0U || trace->ec_receive_ns != 0U)
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
  trace->ec_receive_ns = single_shot_probe::now_ns();
  std::printf("[Probe][EtherCAT] receive_callback master=%d motor=%d ec_receive_ns=%llu "
              "position_actual=%d velocity_actual=%d current_actual=%d\n",
              master_index,
              target_motor,
              static_cast<unsigned long long>(trace->ec_receive_ns),
              motor.position_actual,
              motor.velocity_actual,
              motor.current_actual);
}
} // namespace

void MasterProgram::init(int affinity, int priority, int interval, std::int64_t cycle_time, std::int64_t shiftTime, const std::string &fileName)
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
    // eni模式下直接获取指定的xml文件中的配置
    std::cout << "Use eni xml\n";
    task.load_eni(fileName, cycle_time);
  }
  else
  {
    // esi模式下设置默认的循环周期和同步模式
    std::cout << "Use esi xml\n";
    task.cycle_time(cycle_time, shiftTime);
    task.dc_mode(ecat::dc_mode::master_follow_slave);

    // not_using_dc,
    // master_follow_slave,
    // slave_follow_master,
  }

  // 设置config的回调函数，将在task.start的时候被到用
  task.set_config_callback([&]
                           {

    std::uint16_t slave_count = task.slave_count();
    std::uint16_t slave_pos = 0;




    for (slave_pos = 0; slave_pos < slave_count; slave_pos++)
    { 
      //获取slave对应的运行协议，如果还未设置则默认为0
      auto profile_no = task.profile_no(slave_pos);

    std::cout<<"profile : "<<profile_no<<"    slave_count: "<<slave_count<<std::endl;
      //CiA402协议模式
      if (profile_no == 402)
      {

        auto t = std::chrono::steady_clock::now().time_since_epoch().count() / 1'000'000.0;
        printf("[M1] config_callback enter %.3f ms\n", t);
        // ... 402 伺服初始化 .


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

          axis->master_id = static_cast<std::uint16_t>(master_index_);
          

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
          task.try_register_pdo_entry(axis->target_torque, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6071 + index_offset), 0}); //target_torque
          task.try_register_pdo_entry(axis->torque_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x6077 + index_offset), 0}); //torque_actual_value
          task.try_register_pdo_entry(axis->target_velocity, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x60ff + index_offset), 0}); //target_velocity   
          task.try_register_pdo_entry(axis->velocity_actual_value, slave_pos,
                                      {static_cast<ecat::pdo_index_type>(0x606c + index_offset), 0}); //velocity_actual_value                      
          //添加轴，以及对应的program
          programs.emplace_back(program{axis.get()});
          axes.push_back(std::move(axis));
        }
      }
      else if (profile_no == 401 || profile_no == 5001)
      { 
        auto t = std::chrono::steady_clock::now().time_since_epoch().count() / 1'000'000.0;
        printf("[M0] config_callback enter %.3f ms\n", t);
        // ... 5001 IO 初始化 ...

        std::unique_ptr<IoController> io_ins;
        if (profile_no == 5001)
          io_ins = std::make_unique<IO_example_rw>();
        else
          io_ins = std::make_unique<IO_example_ro>();

        std::vector<ecat::pdo_entry_info> rxEntries = task.get_io_rx_config(slave_pos, 0);

        io_ins->slave_pos = slave_pos;          // ← 就在这里赋值

        // printf("=== new IOController for slave %u, ptr=%p\n", slave_pos, io_ins.get());
        for (auto& pdo : rxEntries) {
          if (pdo.entry_idx.idx == 0)
            continue;
          auto io = std::make_unique<io_data>();
          io->slave_pos = slave_pos;    
          // io->io_idx = pdo.entry_idx.idx;
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
          // io->io_idx = pdo.entry_idx.idx;
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
    ecat::S2SConfig::use().count(); });

  // 设置cycle的回调函数，会在task.start的时候被调用
  // 执行状态机切换以及运动控制
  task.set_receive_callback([&]
                            {
    for (auto &prog : programs)
    {
      prog();
      // struct timeval *tv;
      // if(gettimeofday(tv) )
      // static int cnt = 0;
      // if (++cnt % 1000 == 0)
      //   printf("cycle callback thread ID: %lu\n", std::this_thread::get_id());
    }
    record_receive_probe(master_index_);
    update_master_cycle_status(master_index_, static_cast<int>(task.get_wcstate())); });
}

void MasterProgram::start()
{
  task.start();
}

void MasterProgram::startAsMaster()
{
  task.setAsMaster();
}

bool MasterProgram::startAsSlave(MasterProgram *master)
{
  printf("..................  startAsSlave, period: %d, master period: %d...\n", run_period, master->run_period);
  if (run_period < master->run_period)
  {
    printf("Start ec as slave failed, the period(%d) must >= the master period(%d).\n", run_period, master->run_period);
    return false;
  }

  if (run_period % master->run_period != 0)
  {
    printf("Start ec as slave failed,  period(%d) must be times of master period(%d).\n", run_period, master->run_period);
    return false;
  }

  task.setAsSlave(&master->task, run_period / master->run_period);
  return true;
}

bool MasterProgram::startAsSlave(MasterProgram *master, int num)
{
  printf("..................  startAsSlave, period: %d, master period: %d...\n", run_period, master->run_period);
  if (run_period < master->run_period)
  {
    printf("Start ec as slave failed, the period(%d) must >= the master period(%d).\n", run_period, master->run_period);
    return false;
  }

  if (run_period % master->run_period != 0)
  {
    printf("Start ec as slave failed,  period(%d) must be times of master period(%d).\n", run_period, master->run_period);
    return false;
  }

  task.setAsSlave(&master->task, run_period / master->run_period, num);
  return true;
}

void MasterProgram::wait()
{
  task.wait();
}

void MasterProgram::release()
{
  task.release();
}
