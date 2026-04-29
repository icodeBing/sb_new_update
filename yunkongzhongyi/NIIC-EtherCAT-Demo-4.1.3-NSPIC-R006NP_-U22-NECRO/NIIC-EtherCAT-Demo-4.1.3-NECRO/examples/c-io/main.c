#include "ec_task.h"
#include <getopt.h>
#include <pthread.h>
#include <qiuniu/init.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <assert.h>
#include <time.h>

#include "create_threads.h"
#include "spi_io.h"
#include <signal.h>
#include <sys/types.h>

#define MAX_AXIS_NUM 64
#define MAX_IO_NUM 64

#define contrlword_shutdown(c) (((c) | 0x6) & ~0x81)
#define contrlword_switch_on(c) (((c) | 0x7) & ~0x88)
#define contrlword_disable_voltage(c) ((c) & ~0x82)
#define contrlword_quick_stop(c) (((c) | 0x2) & ~0x84)
#define contrlword_disable_operation(c) (((c) | 0x7) & ~0x88)
#define contrlword_enable_operation(c) (((c) | 0xF) & ~0x80)
#define contrlword_fault_reset(c) ((c) | 0x80)
#define contrlword_new_set_point(c) ((c) | 0x10)
#define contrlword_new_set_point_imm(c) ((c) | 0x30)

static const unsigned char test_table[] = 
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const unsigned int test_dac[] = 
{
  0, 409, 819, 1228, 1638, 2047, 2457, 2866, 3276, 3685, 4095
};

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

enum xml_mode
{
  XML_MODE_ESI = 0,
  XML_MODE_ENI = 1,
  MODE_SPI = 2
};

struct axis_data
{
  uint16_t axis_id;
  uint16_t slave_pos;

  // process data address
  volatile uint16_t *control_word;
  volatile int32_t *target_position;
  volatile int8_t *mode_of_operation;
  volatile int32_t *target_velocity;

  volatile const uint16_t *error_code;
  volatile const uint16_t *status_word;
  volatile const int32_t *position_actual_value;
  volatile const int8_t *mode_of_operation_display;
  volatile const int32_t *velocity_actual_value;

  uint16_t control_word_buffer;
  int32_t target_position_buffer;
  int32_t target_velocity_buffer;
  int8_t mode_of_operation_buffer;

  uint16_t error_code_buffer;
  uint16_t status_word_buffer;
  int32_t position_actual_value_buffer;
  int32_t velocity_actual_value_buffer;
  int8_t mode_of_operation_display_buffer;

  uint16_t old_status_word;

  uint32_t count;
  uint32_t vel;
  uint32_t pos;

  int fault_reset_state;
};

struct io_data
{
  uint16_t io_id;
  uint16_t slave_pos;

  // process data address
  volatile uint16_t *io_address;
  uint8_t io_bit_pos;
  uint32_t count;
};

struct spi_data
{
  spi_file_t fd;              // spi文件句柄

  /* SPI输入命令码
    gpio_out_1: 对应引脚输出状态Byte1 (置0高电平，置1低电平)
    gpio_out_2: 对应引脚输出状态Byte2 (置0高电平，置1低电平)
    DAC_data：  对应DAC数据（设置范围0-4095，对应0V-10V）
    QEI_sample：对应QEI 采样周期
    QEI_value： 对应QEI 比较寄存器数值（脉冲数-1，如399表示一圈一共经历400个脉冲）
  */
  spi_io_write_t nplc_w;      // 数据输出结构体

  /* SPI输入命令码
    gpio_in_1:  对应引脚输入状态Byte1 (有输入为1，无输入为0)
    gpio_in_2:  对应引脚输入状态Byte2 (有输入为1，无输入为0)
    gpio_out_1: 对应引脚输出状态Byte3
    gpio_out_2: 对应引脚输出状态Byte4
    DAC_data：  对应EADC 采样数据
    QEI_sample：对应QEI 脉冲数据
    QEI_value： 对应QEI 方向位（1-表示编码器正向顺时针转动， 2-表示编码器反向逆时针转动）
  */
  spi_io_read_t nplc_r;       // 数据输入结构体

  int priority;               // 线程优先级
  char spi_mask;              // spi硬件编号（0或1）
};

static struct spi_data spi_0, spi_1;

static int axis_count;
static struct axis_data axes[MAX_AXIS_NUM];
int io_input_count, io_output_count;
static struct io_data io_output[MAX_IO_NUM];
static struct io_data io_input[MAX_IO_NUM];
static void usage(const char *program_name)
{
  printf("Usage:\n"
         " -h [ --help ]                     This is EtherCAT Demo program.\n"
         " -f [ --fileName ] arg             Set eni xml, only for eni mode.\n"
         " -c [ --cycleTime ] arg (=1000000) Set cycle time(ns), only for esi "
         "mode.\n"
         " -s [ --shiftTime ] arg (=300000)  Set cycle shift time(ns), only for esi mode. \n"
         " -l [ --log ] arg (=6)             Set log level, (0-6).\n"
         " -o [ --spi_cycle ] arg (=1000000) Set cycle time(ns), only for spi "
         "mode.\n"
         " -p [ --priority ] arg (=50)       Set priority of the realtime "
         "thread.\n"
         " -a [ --affinity ] arg             Set CPU affinity of the realtime "
         "thread.\n"
         " example:\n"
         "         %s \n"
         "         %s -h\n"
         "         %s -f eni.xml\n"
         "         %s -o 1000000\n"
         "         %s -c 500000\n",
         program_name, program_name, program_name, program_name, program_name);
}

static void io_write_bit(struct io_data *io, int val);
static int io_read_bit(struct io_data *io);

// 获取文件句柄
static void open_spi_file()
{
  spi_open_file(&spi_0.fd, DEV_NAME0);
  spi_open_file(&spi_1.fd, DEV_NAME1);

  if (spi_0.fd == -1)
    printf("Fail to open SPI_0. Please check SPI_0 device.\n");
  if (spi_1.fd == -1)
    printf("Fail to open SPI_1. Please check SPI_1 device.\n");
}

// 关闭文件
static void close_spi_file()
{
  if (spi_0.fd != -1)
    spi_close_file(&spi_0.fd);
  if (spi_1.fd != -1)
    spi_close_file(&spi_1.fd);
}

static void set_io_output(struct io_data *io)
{
  io->count++;
  if (io->count < 1000)
    io_write_bit(io, 1);
  else
    io_write_bit(io, 0);
  io->count %= 10000;
}

static void get_io_input(struct io_data *io)
{
  io->count++;
  if (!(io->count % 1000))
  {
    int val = io_read_bit(io);
    printf("read io %u, value is %i\n", io->io_id, val);
  }
  io->count %= 10000;
}

static int axis_enable(struct axis_data *axis);

// Motion control
static void axis_control_demo(struct axis_data *axis)
{
  // Start to move after the axis status is operation_enable.
  if (axis_enable(axis))
  {
    axis->target_velocity_buffer = 1310720;
    // The servo will cyclically rotate forward and reverse.
    if (axis->count < 100 || axis->count >= 500 && axis->count < 600)
    {
      axis->vel += 10;
      axis->pos += axis->vel;
      axis->count += 1;
    }
    else if ((axis->count >= 100 && axis->count < 200) ||
             (axis->count >= 400 && axis->count < 500))
    {
      axis->count += 1;
      axis->pos += axis->vel;
    }
    else if (axis->count >= 200 && axis->count < 400)
    {
      axis->vel -= 10;
      axis->pos += axis->vel;
      axis->count += 1;
    }

    axis->count %= 600;

    axis->target_position_buffer = axis->pos;
  }
  else
  {
    // When axis is not in opreation_enable, init the axis info.
    axis->count = 0;
    axis->vel = 0;
    axis->pos = axis->position_actual_value_buffer;
  }
}

static void config_cb(void *user_data)
{
  ec_task_t *task = user_data;
  int i;
  int slave_count;

  slave_count = ec_task_get_slave_count(task);

  for (i = 0; i < slave_count; ++i)
  {
    int profile_no = ec_task_get_profile_no(task, i);
    // Handler the 402 servo.
    if (profile_no == 402)
    {
      int n_axis_in_slave = 1;
      int slot_pos;
      int slots_count = ec_task_slave_slots_count(task, i);
      int slot_pdo_increment = ec_task_slave_slot_pdo_increment(task, i);
      int slot_index_increment = ec_task_slave_slot_index_increment(task, i);

      // Handler the multi-axis.
      if (slots_count > 0)
      {
        assert(slot_pdo_increment != -1);
        assert(slot_index_increment != -1);
        n_axis_in_slave = slots_count;
      }
      assert(!(slots_count < 0));

      // Register PDO entry for every axis.
      for (slot_pos = 0; slot_pos < n_axis_in_slave; ++slot_pos)
      {
        const int index_offset = slot_pos * slot_index_increment;
        uint16_t abort_code = 0;

        axes[axis_count].axis_id = axis_count;
        axes[axis_count].slave_pos = i;

        ec_task_register_pdo_entry(task, &axes[axis_count].error_code,
                                   axes[axis_count].slave_pos,
                                   0x603f + index_offset, 0, 0, 0);
        ec_task_register_pdo_entry(task, &axes[axis_count].control_word,
                                   axes[axis_count].slave_pos,
                                   0x6040 + index_offset, 0, 0, 0);
        ec_task_register_pdo_entry(task, &axes[axis_count].status_word,
                                   axes[axis_count].slave_pos,
                                   0x6041 + index_offset, 0, 0, 0);
        ec_task_register_pdo_entry(task, &axes[axis_count].mode_of_operation,
                                   axes[axis_count].slave_pos,
                                   0x6060 + index_offset, 0, 0, 0);

        ec_task_register_pdo_entry(
            task, &axes[axis_count].mode_of_operation_display,
            axes[axis_count].slave_pos, 0x6061 + index_offset, 0, 0, 0);
        ec_task_register_pdo_entry(task, &axes[axis_count].target_position,
                                   axes[axis_count].slave_pos,
                                   0x607a + index_offset, 0, 0, 0);
        ec_task_register_pdo_entry(
            task, &axes[axis_count].position_actual_value,
            axes[axis_count].slave_pos, 0x6064 + index_offset, 0, 0, 0);

         ec_task_register_pdo_entry(task, &axes[axis_count].target_velocity,
                                    axes[axis_count].slave_pos, 0x60FF, 0, 0, 0);
         ec_task_register_pdo_entry(task,  &axes[axis_count].velocity_actual_value,
                                    axes[axis_count].slave_pos, 0x606c, 0, 0, 0);

        ++axis_count;
      }
    }
    else if (profile_no == 5001)
    {
      int rx_pdos_size = ec_task_rx_pdos_size(task, i);
      int tx_pdos_size = ec_task_tx_pdos_size(task, i);
      printf("Test rx size is %d, tx size is %d\n", rx_pdos_size, tx_pdos_size);

        for (int j = 0; j < tx_pdos_size; ++j)
        {
          io_input[io_input_count].slave_pos = i;
          io_input[io_input_count].io_id = io_input_count;
          ec_task_register_pdo_entry(task, &io_input[io_input_count].io_address,
                                     io_input[io_input_count].slave_pos, (0x6000 + j * 0x10), 
                                     1, &io_input[io_input_count].io_bit_pos, 0);
          io_input_count++;
        }
        for (int j = 0; j < rx_pdos_size; ++j)
        {
          io_output[io_output_count].slave_pos = i;
          io_output[io_output_count].io_id = io_output_count;
        ec_task_register_pdo_entry(task, &io_output[io_output_count].io_address,
                                   io_output[io_output_count].slave_pos, (0x7000+j*0x10),
                                   1, &io_output[io_output_count].io_bit_pos, 0);
        io_output_count++;
        }
    }
  }

  
  ec_task_set_interval(task, axis_count);
  printf("axis_count:%d\n", axis_count);
}

// Get the PDO data
static void axis_data_update(struct axis_data *axis)
{
  if (axis->control_word)
    *axis->control_word = axis->control_word_buffer;
  if (axis->target_position)
    *axis->target_position = axis->target_position_buffer;
  if (axis->mode_of_operation)
    *axis->mode_of_operation = axis->mode_of_operation_buffer;
  if (axis->target_velocity)
    *axis->target_velocity = axis->target_velocity_buffer;

  if (axis->error_code)
    axis->error_code_buffer = *axis->error_code;
  if (axis->status_word)
    axis->status_word_buffer = *axis->status_word;
  if (axis->position_actual_value)
    axis->position_actual_value_buffer = *axis->position_actual_value;
  if (axis->mode_of_operation_display)
    axis->mode_of_operation_display_buffer = *axis->mode_of_operation_display;
  if (axis->velocity_actual_value)
    axis->velocity_actual_value_buffer = *axis->velocity_actual_value;
}
static int b = 1000;
static int c = 0;
static void receive_cb(void *user_data)
{
  int i;

  for (i = 0; i < axis_count; ++i)
  {
    axis_data_update(&axes[i]);
  }
}

static void send_cb(void *user_data)
{
  int i;

  for (i = 0; i < axis_count; ++i)
  {
    axis_control_demo(&axes[i]);
  }
  for (i = 0; i < io_output_count; ++i)
  {
    set_io_output(&io_output[i]);
  }
  for (i = 0; i < io_input_count; ++i)
  {
    get_io_input(&io_input[i]);
  }
  
  ec_pwm_config(1, b);
  ec_pwm_config(2, b);
  ec_pwm_config(3, b);
  ec_pwm_config(4, b);

}

// SPI_0的回调函数
int set_spi_0()
{
  static int j = 0;
  j++;

  // if (j == 1){
  //   spi_0.nplc_w.gpio_out_1 = 0x00;
  //   spi_0.nplc_w.gpio_out_2 = 0x00;
  // }

  if (j % 1000 == 0){   // 每1s设置一次 
    if (spi_read(spi_0.fd, &spi_0.nplc_r)){
      return 1;
    }
    __RT(printf("SPI_0   in: %02x%02x, out: %02x%02x, EADC: %d, QEI: %d, direction: %d\n", 
                  spi_0.nplc_r.gpio_in_1, spi_0.nplc_r.gpio_in_2, spi_0.nplc_r.gpio_out_1, spi_0.nplc_r.gpio_out_2, 
                  spi_0.nplc_r.EDAC_data, spi_0.nplc_r.QEI_data, spi_0.nplc_r.QEI_direction));

    static int c0 = 0;
    spi_0.nplc_w.gpio_out_1 = test_table[c0];//01
    spi_0.nplc_w.gpio_out_2 = test_table[c0+16];//00
    // spi_0.nplc_w.gpio_out_1 = ~spi_0.nplc_w.gpio_out_1;
    // spi_0.nplc_w.gpio_out_2 = ~spi_0.nplc_w.gpio_out_2;
    static int dac0 = 0;
    if (j % 10000 == 0)
    {
      spi_0.nplc_w.DAC_data = test_dac[dac0];
      dac0++;
    }
    if (dac0 == 11){dac0 = 0;}
    spi_0.nplc_w.QEI_sample = 0;
    spi_0.nplc_w.QEI_value = 399;
    c0++;
    if (c0 == 16){c0 = 0;}
    
    spi_write(&spi_0.fd, &spi_0.nplc_w);
  }
  return 0;
}

// SPI_1的回调函数
int set_spi_1()
{
  static int k = 0;
  k++;

  // if (k == 1){
  //   spi_1.nplc_w.gpio_out_1 = 0xff;
  //   spi_1.nplc_w.gpio_out_2 = 0xff;
  // }
  if (k % 1000 == 0){
    if (spi_read(spi_1.fd, &spi_1.nplc_r)){
      return 1;
    }
    __RT(printf("SPI_1   in: %02x%02x, out: %02x%02x, EADC: %d, QEI: %d, direction: %d\n\n", 
                  spi_1.nplc_r.gpio_in_1, spi_1.nplc_r.gpio_in_2, spi_1.nplc_r.gpio_out_1, spi_1.nplc_r.gpio_out_2, 
                  spi_1.nplc_r.EDAC_data, spi_1.nplc_r.QEI_data, spi_1.nplc_r.QEI_direction));

    static int c1 = 0;
    spi_1.nplc_w.gpio_out_1 = test_table[c1];
    spi_1.nplc_w.gpio_out_2 = test_table[c1+16];
    // spi_1.nplc_w.gpio_out_1 = ~spi_1.nplc_w.gpio_out_1;
    // spi_1.nplc_w.gpio_out_2 = ~spi_1.nplc_w.gpio_out_2;

    static int dac1 = 0;
    if (k % 10000 == 0)
    {
      spi_1.nplc_w.DAC_data = test_dac[dac1];
      dac1++;
    }
    if (dac1 == 11){dac1 = 0;}

    spi_1.nplc_w.QEI_sample = 300;
    spi_1.nplc_w.QEI_value = 0;
    c1++;
    if (c1 == 16){c1 = 0;}

    spi_write(&spi_1.fd, &spi_1.nplc_w);
  }
  return 0;
}

ec_task_t *tt;
bool ethercat = false;

void signal_hander(int sig)
{
  signal(SIGINT, SIG_DFL);
  if (sig == SIGINT){
    if (ethercat)
      ec_task_break(tt);
    spi_stop();
  }
}

int main(int argc, char *argv[])
{
  signal(SIGINT, signal_hander);
  ec_task_t *task;
  int err;
  int option_index = 0, c;
  cpu_set_t cpus;
  int xml_mode = XML_MODE_ESI;
  int64_t cycle_time = 1000000;
  int64_t shiftTime = 0;
  char *path;
  int priority = 80;
  int affinity = 1;
  int dc_mode = 1;
  long spi_cycle = 1000000;

  static struct option long_options[] = {
      {"help", no_argument, 0, 'h'},
      {"fileName", required_argument, 0, 'f'},
      {"cycleTime", required_argument, 0, 'c'},
      {"shiftTime", required_argument, 0, 's'},
      {"log", required_argument, 0, 'l'},
      {"priority", required_argument, 0, 'p'},
      {"affinity", required_argument, 0, 'a'},
      {"dc_mode", required_argument, 0, 'd'},
      {"spi_cycle", optional_argument, 0, 'o'},
      {0, 0, 0, 0},
  };

  qiuniu_init();
  ec_pwm_init();
  ec_pwm_open(1);
  ec_pwm_open(2);
  ec_pwm_open(3);
  ec_pwm_open(4);

  open_spi_file();

  if (mlockall(MCL_FUTURE | MCL_CURRENT) == -1)
  {
    perror("failed to lock memory\n");
    return 1;
  }

  while ((c = getopt_long(argc, argv, "f:c:hl:p:a:d:s::", long_options,
                          &option_index)) != -1)
  {
    switch (c)
    {
    case 'f':
    {
      xml_mode = XML_MODE_ENI;
      path = optarg;
      break;
    }
    case 'c':
    {
      cycle_time = atoi(optarg);
      break;
    }
    case 's':
    {
      shiftTime = atoi(optarg);
      break;
    }
    case 'h':
    {
      usage(argv[0]);
      return 0;
    }
    case 'l':
    {
      ec_task_set_log_level(atoi(optarg));
      break;
    }
    case 'p':
    {
      priority = atoi(optarg);
      break;
    }
    case 'a':
    {
      affinity = atoi(optarg);
      break;
    }
    case 'd':
    {
      dc_mode = atoi(optarg);
      break;
    }
    case 'o':
    {
      xml_mode = MODE_SPI;
      if (optarg)
        spi_cycle = (long)atoi(optarg);
      break;
    }
    default:
      usage(argv[0]);
      return 0;
    }
  }

  ethercat = ethercat_mode(xml_mode);
  if (ethercat){
  
  task = ec_task_new(0);
  tt = task;

  ec_task_set_priority(task, priority);

  if (affinity != -1)
  {
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(affinity, &cpus);

    ec_task_set_affinity(task, &cpus, sizeof(cpus));
  }

  if (xml_mode == XML_MODE_ENI)
  {
    if (path == NULL)
    {
      ec_task_finalize(task);
      usage(argv[0]);
      return 1;
    }
    printf("Use eni mode\n");
    ec_task_load_eni(task, path, &cycle_time);
    if (dc_mode == 1) {
      ec_task_set_dc_mode(task, EC_DC_MASTER_FOLLOW_SLAVE);
    }
    else if (dc_mode == 0){
      ec_task_set_dc_mode(task, EC_DC_UNUSED);
    }
    else {
      printf("Set dc mode error\n");
      return 1;
    }
  }
  else if (xml_mode == XML_MODE_ESI)
  {
    if (cycle_time == 0)
    {
      ec_task_finalize(task);
      usage(argv[0]);
      return 1;
    }
    printf("Use esi mode\n");
    if (dc_mode == 1) {
      ec_task_set_dc_mode(task, EC_DC_MASTER_FOLLOW_SLAVE);
      ec_task_set_cycle_time(task, cycle_time, shiftTime);
    }
    else if (dc_mode == 0){
      ec_task_set_dc_mode(task, EC_DC_UNUSED);
    }
    else {
      printf("Set dc mode error\n");
      return 1;
    }
  }

  ec_task_set_config_callback(task, config_cb, task);
  ec_task_set_receive_callback(task, receive_cb, task);
  ec_task_set_send_callback(task, send_cb, task);
  }
  else{
    printf("Use SPI mode\n");
    spi_create_pthread(90, (char)2, spi_cycle);
  }

  callback_spi_0(&set_spi_0);
  callback_spi_1(&set_spi_1);

  if (ethercat){
  err = ec_task_start(task);
  if (err != 0)
  {
    printf("Failed to start ethercat task: %s\n", strerror(-err));
    ec_task_finalize(task);
    return 1;
  }
  }

  // SPI线程
  if (spi_0.fd != -1)
    spi_create_pthread(spi_0.priority = 60, spi_0.spi_mask = 0, spi_cycle);
  if (spi_1.fd != -1)
    spi_create_pthread(spi_1.priority = 60, spi_1.spi_mask = 1, spi_cycle);
  
  if (ethercat){
    err = ec_task_wait(task);
    ec_task_finalize(task);
  }
  else{
    spi_close_pthread((char)2);
  }

  if (spi_0.fd != -1)
    spi_close_pthread(spi_0.spi_mask);
  if (spi_1.fd != -1)
    spi_close_pthread(spi_1.spi_mask);
  spi_end(spi_0.fd, &spi_0.nplc_w);
  spi_end(spi_1.fd, &spi_1.nplc_w);
  close_spi_file();

  ec_pwm_close();
  return err == 0;
}

// Get the axis state by status_word
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

const static const char *state_strings[] = {
    [switch_on_disable] = "switch on disable",
    [ready_to_switch_on] = "ready to switch on",
    [switched_on] = "switched on",
    [operation_enable] = "operation enable",
    [quick_stop_active] = "quick stop active",
    [fault_reaction_active] = "fault reaction active",
    [fault] = "fault",
};

// Switch the axis state to string for print.
static const char *axis_state_string(int state)
{
  if (state > 7 || state < 0)
    return "uknown state";

  return state_strings[state];
}

// Switch axis state.
static int axis_enable(struct axis_data *axis)
{
  uint16_t status = axis->status_word_buffer;
  axis_state_t s = get_axis_state(status);
  int ret = 0;

  switch (s)
  {
  case (no_ready_to_switch_on):
  case (switch_on_disable):
    axis->control_word_buffer = contrlword_shutdown(0);
    break;
  case (ready_to_switch_on):
    axis->control_word_buffer = contrlword_switch_on(0);
    break;
  case (switched_on):
    axis->control_word_buffer = contrlword_enable_operation(0);
    if (axis->mode_of_operation)
      axis->mode_of_operation_buffer = op_mode_csp;
    break;
  case (operation_enable):
    ret = 1;
    break;
  case (quick_stop_active):
  case (fault_reaction_active):
    break;
  case (fault):
    axis->control_word_buffer = contrlword_fault_reset(0);
    // if (axis->error_code)
    //   __RT(printf("Axis %d error code: %#04x\n", axis->axis_id,
    //          axis->error_code_buffer));
    break;
  }

  if (s != operation_enable)
  {
    axis->target_position_buffer = axis->position_actual_value_buffer;
  }

  if (get_axis_state(axis->old_status_word) != s)
  {
    __RT(printf("Axis %d changed from %s to %s\n", axis->axis_id,
           axis_state_string(get_axis_state(axis->old_status_word)),
           axis_state_string(s)));
    axis->old_status_word = status;
  }

  return ret;
}

void io_write_bit(struct io_data *io, int val)
{
    if (val)
      *(uint16_t *)io->io_address |=  (1 << io->io_bit_pos);
    else
      *(uint16_t *)io->io_address &= ~(1 << io->io_bit_pos);
}

int io_read_bit(struct io_data *io)
{
   return (*(uint16_t *)io->io_address >> io->io_bit_pos) & 0x01;
}
