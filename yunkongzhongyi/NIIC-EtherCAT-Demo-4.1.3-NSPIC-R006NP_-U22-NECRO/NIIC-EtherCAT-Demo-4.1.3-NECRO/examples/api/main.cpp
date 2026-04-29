#include <iostream>
#include <qiuniu/init.h>
#include <vector>
#include <boost/program_options.hpp>
#include <signal.h>
#include <ecat/master.hpp>
#include "ecat/rokae/extra.hpp"
#include <cmath>
#include <cstring>
#include <tuple>
#include <unordered_map>
#include <iomanip>
#include <unistd.h>
#include <fstream>
#include <sstream>
#include "ecat/run_time_stat.hpp"
#include "ecat/rokae/extra.hpp"
#include "ecat/niic_api.hpp"
#include "ecat/eni_config.hpp"

using namespace ecat;

static void usage(const char * program_name)
{
  printf(" example:\n"
         "         %s -h\n"
         "         %s -m -r\n"
         "         %s -s -w -p 0 -a 2\n"
         "         %s -e -w -p 0 -i /home/root/SSC-Device-401.bin\n"
         "         %s -f -w -p 0 -n GPIO_INT -i /home/root/GPIO_INT.bin\n",
         program_name, program_name, program_name, program_name, program_name);
}

int main(int argc, const char **argv)
{
  qiuniu_init();

  boost::program_options::options_description opts("all option");
  boost::program_options::variables_map vm;
  int slave_position;
  std::string file_name;
  std::string file_path;
  int al_state;
  std::string idx;
  uint16_t sub_idx;
  uint32_t param;
  int logLevel;

  opts.add_options()
      ("help,h", "This is EtherCAT Demo Tools.")
      ("read,r", "Read slave function.")
      ("write,w", "Write slave function.")
      ("foe,f", "foe function.")
      ("sdo,o", "SDO function.")
      ("eeprom,e", "eeprom function.")
      ("slave_state,s", "slave_state function.")
      ("master_state,m", "master_state function.")
      ("rescan,c", "rescan slaves.")
      ("file_name,n", boost::program_options::value<std::string>(&file_name), "Set file name to be writen.")
      ("file_path,t", boost::program_options::value<std::string>(&file_path), "Set file path to be writen.")
      ("index,i", boost::program_options::value<std::string>(&idx), "SDO index.")
      ("sub_index,b", boost::program_options::value<uint16_t>(&sub_idx)->default_value(0), "SDO sub index.")
      ("param,x", boost::program_options::value<uint32_t>(&param)->default_value(0), "SDO parameter.")
      ("log_level,l", boost::program_options::value<int>(&logLevel)->default_value(6), "0(all) - 6(none)")
      ("slave_position,p", boost::program_options::value<int>(&slave_position)->default_value(0), "Set slave_position, default 0.")
      ("al_state,a", boost::program_options::value<int>(&al_state)->default_value(2), "Set al_state, default preop.");
  
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

  // 创建ECAT任务，自动创建主站对象
  ecat::task task_(0);

  // if (task_.get_master_ptr()->get_info().slave_count <= slave_position || slave_position < 0)
  // {
  //   std::cout << "Only " << task_.get_master_ptr()->get_info().slave_count << " slaves on the net\n";
  //   return 1;
  // }

  master* master_ = task_.get_master_ptr();
  int slave_count = master_->get_info().slave_count;
  // printf("info slave count: %d\n", slave_count);

  std::uint8_t state_ = (uint8_t)al_state;

  // 从站EC状态
  if (vm.count("slave_state"))
  {
    // 读取
    if (vm.count("read"))
    {
      int ec = get_Slave_State(&task_, slave_position, state_);
      if (ec)
      {
        perror("get_Slave_State");
        return -1;
      }
      std::string sls = to_strng_al_state(state_);
      std::cout << "slave: " << slave_position << ", state: " << sls.c_str() << std::endl; 
    }
    // 设置
    else if (vm.count("write"))
    {
      int ec = set_Slave_State(&task_, slave_position, state_);
      if (ec)
      {
        perror("set_Slave_State");
        return -1;
      }
      usleep(100000);
      ec = get_Slave_State(&task_, slave_position, state_);
      std::string sls = to_strng_al_state(state_);
      std::cout << "set slave "<< slave_position << " state [" << sls.c_str() << "] successfully" << std::endl; 
    }
    return 0;
  }

  // 主站EC状态
  if (vm.count("master_state"))
  {
    // 读取
    if (vm.count("read"))
    {
      int ec = get_Master_State(&task_, state_);
      if (ec)
      {
        perror("get_Master_State");
        return -1;
      }
      std::string mas = to_strng_al_state(state_);
      std::cout << "master_state: " << mas << std::endl; 
    }
    // 设置
    else if (vm.count("write"))
    {
      int ec = set_Master_State(&task_, state_);
      if (ec)
      {
        perror("set_Master_State");
        return -1;
      }
      usleep(100000);
      ec = get_Master_State(&task_, state_);
      std::string mas = to_strng_al_state(state_);
      std::cout << "set master state [" << mas.c_str() << "] successfully" << std::endl; 
    }
    return 0;
  }

  // EEPROM
  if (vm.count("eeprom"))
  {
    //读取
    if (vm.count("read"))
    {
      // 创建结构体保存eeprom信息
      struct ec_slave_sii_t info;
      info.slave_position = slave_position;
      info.offset = 0;

      // 执行此函数会申请内存来保存读到的eeprom
      int ec = read_eeprom(&task_, &info);
      if (ec)
      {
        perror("read_eeprom");
        return -1;
      }

      // 打印
      printf(" sii_nwords: %d\n", info.nwords);
      for (int i = 0; i < info.nwords; i++) 
      {
        if (i % 8 == 0) 
        {
          printf("\n");
        }
        printf(" %04x", info.words[i]);
      }
      printf("\n");

      // 释放内存
      if (info.words != nullptr)
      {
        delete[] info.words;
        info.words = nullptr;
      }
    }
    // 写入
    else if (vm.count("write"))
    {
      // 创建结构体，并打开文件保存之
      struct ec_slave_sii_t info;
      std::ifstream file;
      std::ostringstream tmp;
      file.open(file_path.c_str(), std::ifstream::in | std::ifstream::binary);
      if (file.fail())
      {
        printf("Fail to open %s\n", file_path.c_str());
        return -1;
      }
      tmp << file.rdbuf();
      std::string const &contents = tmp.str();
      std::cout << "Read " << contents.size() << " bytes of SII data." << std::endl;
      if (!contents.size() || contents.size() % 2)
      {
        printf("Error bin file\n");
        return -1;
      }
      info.nwords = contents.size() / 2;
      printf("nwords: %d\n", info.nwords);
      info.words = new uint16_t[info.nwords];
      contents.copy((char *) info.words, contents.size());
      file.close();

      info.offset = 0;
      info.slave_position = slave_position;
        
      // 成功返0，失败返错误码
      int ec = ecat::write_eeprom(&task_, &info);
      if (ec)
      {
        perror("write_eeprom");
        return -1;
      }

      // 重新扫描从站
      ec = ecat::rescan_Slaves(&task_);
      if (ec)
      {
        perror("rescan_Slaves");
        return -1;
      }

      if (info.nwords > 0)
      {
        delete [] info.words;
        info.words = nullptr;
      }
    }
    return 0;
  }

  // FoE
  if (vm.count("foe"))
  {
    int ec;
    // 切到BOOT状态
    ec = ecat::set_Slave_State(&task_, slave_position, (uint8_t)3);
    if (ec)
    {
      perror("set_Slave_State");
      return -1;
    }
    usleep(100000);
    ec = get_Slave_State(&task_, slave_position, state_);
    if (ec)
    {
      perror("get_Slave_State");
      return -1;
    }
    if (state_ != 3)
    {
      printf("set_Slave_State, state_!=BOOT\n");
      return -1;
    }

    // 读取
    if (vm.count("read"))
    {
      // 创建结构体保存foe信息
      struct ec_slave_foe_t info;
      info.offset = 0;
      info.buffer_size = 0xaa00; // 自定义用于保存foe的内存大小
      info.slave_position = slave_position;
      info.password = 0;
      strncpy(info.file_name, file_name.c_str(), file_name.size()); // 要读取的文件名称

      // 执行此函数会申请内存来保存读到的文件
      int a = ecat::read_foe(&task_, &info, file_name.size());
      printf("----%08x\n", a);
      if (a != 0)
      {
        perror("read_foe");
      }

      // 打印
      printf(" data_size: %ld\n", info.data_size);
      for (int i = 0; i < info.data_size; i++) 
      {
        if (i % 16 == 0) 
        {
          printf("\n");
        }
        printf(" %02x", info.buffer[i]);
      }
      printf("\n");

      // 释放内存
      if (info.data_size)
      {
        delete[] info.buffer;
        info.buffer = nullptr;
      }
    }
    // 写入
    else if (vm.count("write"))
    {
      struct ec_slave_foe_t info;

      // 打开文件
      std::ifstream file;
      std::ostringstream tmp;
      file.open(file_path.c_str(), std::ifstream::in | std::ifstream::binary);
      if (file.fail())
      {
        printf("Fail to open %s\n", file_path.c_str());
        return -1;
      }
      tmp << file.rdbuf();
      std::string const &contents = tmp.str();
      std::cout << "Read " << contents.size() << " bytes of FoE data." << std::endl;
      info.buffer_size = contents.size();
      printf("buffer_size: %ld\n", info.buffer_size);
      info.buffer = new uint8_t[info.buffer_size];
      contents.copy((char *) info.buffer, contents.size());
      file.close();

      info.offset = 0;
      info.slave_position = slave_position;
      info.password = 0;
      strncpy(info.file_name, file_name.c_str(), file_name.size());

      int a = ecat::write_foe(&task_, &info, file_name.size());
      if (a != 0)
      {
        std::cout << a << ", error\n";
      }

      if (info.data_size)
      {
        delete[] info.buffer;
        info.buffer = nullptr;
      }
    }

    // 状态切回
    ec = ecat::set_Slave_State(&task_, slave_position, (uint8_t)2);
    if (ec)
    {
      perror("set_Slave_State");
      return -1;
    }
    return 0;
  }

  // SDO
  if (vm.count("sdo") && vm.count("index"))
  {
    std::cout << "Start to ECAT Configure...\n";
    task_.set_configure(logLevel);
    ecat::sdo_idx index = {(uint16_t)stoi(idx, 0, 16), sub_idx};
    std::cout << "Start to SDO...\n";
    ecat::sdo_request* sdo_ = nullptr;
    int ec = ecat::create_sdo(sdo_, &task_, slave_position, index, sizeof(param), false);
    ecat::sdo_request* sdo_1 = nullptr;
    ec = ecat::create_sdo(sdo_1, &task_, slave_position, {0x2030, 0}, sizeof(param), false);
    // printf("----%08x\n", ec);
    if (ec)
    {
      perror("create_sdo");
      return -1;
    }
    if (vm.count("read"))
    {
      while (1)
      {
        ec = ecat::read_sdo(sdo_, 0);
        ec = ecat::read_sdo(sdo_1, 0);
        if (ec)
        {
          perror("read_sdo");
          break;
        }
        if (sdo_->state() == ecat::request_state::success)
        {
          param = sdo_->data<int32_t>();
          printf("read_sdo: %d\n", param);
          break;
        }
        else if (sdo_->state() == ecat::request_state::error)
        {
          printf("sdo_state: %d\n", static_cast<uint32_t>(sdo_->state()));
          break;
        }
      }
    }
    else if (vm.count("write"))
    {
      sdo_->data<uint32_t>(param);
      ec = ecat::write_sdo(sdo_, 0);
      if (ec)
      {
        perror("write_sdo");
      }
      if (sdo_->state() == ecat::request_state::error)
      {
        printf("sdo_state: %d\n", static_cast<uint32_t>(sdo_->state()));
        return -1;
      }
      printf("Done.\n");
    }
    return 0;
  }
  
  // rescan
  if (vm.count("rescan"))
  {
    int ec = ecat::rescan_Slaves(&task_);
    if (ec != 0)
    {
      perror("rescan_Slaves");
    }
    return 0;
  }


  // unsigned char dda[2];
  // master_->reg_read(0, 0x0130, 1, dda);
  // printf("%02x\n", dda[0]);
  try
  {
    unsigned char dda[2];
    master_->reg_read(0, 0x0130, 1, dda);
    printf("0x0130 : 0x%04x\n", dda[0]);

    ////////////////////////////////

    unsigned char reg_test_1[2];
    master_->reg_read(0, 0x0420, 2, reg_test_1);
    // printf("test_1 0x0420 : 0x%x%x\n", reg_test_1[1], reg_test_1[0]);
    uint16_t reg_test_1n = *(uint16_t*)reg_test_1;
    printf("info test_1 0x0420 : 0x%x\n", reg_test_1n);
    ////////////////////////////////

    unsigned char reg_test_2[2];
    reg_test_2[0] = 0xfa; // [0]
    reg_test_2[1] = 0xfb; // [1]
    master_->reg_write(0, 0x0420, 2, reg_test_2);

    ////////////////////////////////

    unsigned char reg_test_3[2]{ 0 };
    master_->reg_read(0, 0x0420, 2, reg_test_3);
    // printf("x 0x0420 : 0x%x\n", reg_test_3[1]);
    // printf("  0x0420 : 0x%x\n", reg_test_3[0]);
    uint16_t reg_test_3n = *(uint16_t*)reg_test_3;
    printf("info test_3 0x0420 : 0x%x\n", reg_test_3n);
    ////////////////////////////////

    uint16_t reg_test_4 = 0xf2f1;
    master_->reg_write(0, 0x0420, 2, (unsigned char*)&reg_test_4);

    ////////////////////////////////

    unsigned char reg_test_5[2]{ 0 };
    master_->reg_read(0, 0x0420, 2, reg_test_5);
    // printf("x 0x0420 : 0x%x\n", reg_test_5[1]);
    // printf("  0x0420 : 0x%x\n", reg_test_5[0]);
    uint16_t reg_test_5n = *(uint16_t*)reg_test_5;
    printf("info test_5 0x0420 : 0x%x\n", reg_test_5n);
    ////////////////////////////////

    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }

  slave_info slInfo = master_->get_slave_info(0);
  printf("desc: %d, next_slave: %d\n", (int)slInfo.ports.at(0).desc, slInfo.ports.at(0).next_slave);
  printf("vendor: %x, product_code: %x\n", slInfo.id.vendor_id, slInfo.id.product_code);
  printf("alias: %x, device_index: %x\n", slInfo.alias, slInfo.device_index);
  // ecat::master_state tmp = master_->state();
  // printf("%d\n", tmp.slaves_responding);

  ecat::eni_config enInfo(file_path);
  std::optional<ecat::slave_eni_config_info> info = enInfo.get_slave_config_by_index(0);
  printf("ethercat_addr: %d\n", info.value().ethercat_addr);
  // printf("rx_len: %d, tx_len: %d\n", info.value().rx_offset.byte, info.value().tx_offset.byte);

  return 0;
}
