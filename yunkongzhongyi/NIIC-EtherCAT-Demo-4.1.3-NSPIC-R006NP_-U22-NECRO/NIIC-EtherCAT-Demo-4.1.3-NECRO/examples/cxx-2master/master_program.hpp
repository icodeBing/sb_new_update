#pragma once

#include "axis.hpp"
#include <vector>
#include "ecat/task.hpp"
#include "power.hpp"
#include "move_demo.hpp"
#include "io.hpp"

struct program
{
  axis_data *axis;
  power power_;
  move_demo move_;

  void init() {}

  void operator()() {
    power_.axis = axis;
    power_.enable = true;

    move_.axis = axis;
    move_.execute = power_.status;

    power_.on_cycle();
    move_.on_cycle();
  }
};

struct program_input
{
  io_data *io;
  int count = 0;
  void init() {}
  void operator()() {
    count ++;
    if (count < 1000)
      *io->io_address = 0xffff;
    else
      *io->io_address = 0;
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
    if (!(count % 1000))
    {
      uint16_t val = *io->io_address;
      std::cout << "read slave: " << io->slave_pos << "io: " << io->io_id << " value is " << val << "\n";
    }
    count %= 10000;
  }
};

class MasterProgram
{ 
public:
    MasterProgram(int master_index);
    ~MasterProgram();

    void init(int affinity, int priority, int interval, std::int64_t cycle_time, std::int64_t shiftTime, const std::string& fileName);
    void start();
    void startAsMaster();
    bool startAsSlave(MasterProgram* master);
    void wait();
    void release();

    int run_period = 1000000;
    ecat::task task;
    int axis_count = 0;
    std::vector<std::unique_ptr<axis_data>> axes;
    std::vector<std::unique_ptr<io_data>> io_output;
    std::vector<std::unique_ptr<io_data>> io_input;
    std::vector<std::function<void ()>> programs;

};
