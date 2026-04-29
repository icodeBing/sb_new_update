#include <qiuniu/init.h>
#include <signal.h>

#include <ecat/task.hpp>
#include <ecat/types.hpp>
#include <iostream>

static ecat::task* task_ptr;

static void signal_handler(int sig)
{
    signal(sig, SIG_DFL);
    // 必须放在第一行, 否则 coredump 堆栈显示的地址会对应不上

    if (task_ptr)
    {
        // 退出任务
        task_ptr->break_();
    }

    // exit(0); // bad
}

int main()
{
    signal(SIGINT, signal_handler);

    qiuniu_init();

    int master_index = 0;
    int affinity = 1;
    int priority = 60;

    std::int64_t cycle_time = 1000000;
    std::int32_t shift_time = 0;

    std::string eni_file_path = "/home/niic/dolvin/NIIC_ENI_M0.xml";

    uint32_t cycle_count = 0;

    ecat::task task = ecat::task(master_index);
    task_ptr = &task;

    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(affinity, &cpus);

    // 配置实时性
    task.cpu_affinity(&cpus, sizeof(cpus));
    task.priority(priority);

    // eni mode
    // /*
    {
        // 加载 ENI 文件
        task.load_eni(eni_file_path, cycle_time);
    }
    // */

    // esi mode
    /*
    {
        task.cycle_time(cycle_time, shift_time);
        task.dc_mode(ecat::dc_mode::master_follow_slave);
    }
    // */

    volatile uint16_t* control_word;
    volatile int8_t* mode_of_operation;
    volatile int32_t* target_position;
    volatile int32_t* target_velocity;
    volatile int16_t* target_torque;

    volatile const uint16_t* error_code;

    volatile const uint16_t* status_word;
    volatile const int8_t* mode_of_operation_display;
    volatile const int32_t* position_actual_value;
    volatile const int32_t* velocity_actual_value;
    volatile const int16_t* torque_actual_value;

    // clang-format off

    // 设置 config 的回调函数, 将在 task.start 执行后调用 具体在从站配置 Pdo 映射后
    task.set_config_callback(
        [&]
        {
            int slave_pos = 0;

            // error code
            task.try_register_pdo_entry(error_code, slave_pos,
                                        {0x603f, 0});  // error code

            // control & status
            task.try_register_pdo_entry(control_word, slave_pos,
                                        {0x6040, 0});  // control word;
            task.try_register_pdo_entry(status_word, slave_pos,
                                        {0x6041, 0});  // status word

            // mode of operation
            task.try_register_pdo_entry(mode_of_operation, slave_pos,
                                        {0x6060, 0});  // mode of operation
            task.try_register_pdo_entry(mode_of_operation_display, slave_pos,
                                        {0x6061, 0});  // mode of operation display

            // position
            task.try_register_pdo_entry(target_position, slave_pos,
                                        {0x607a, 0});  // target position
            task.try_register_pdo_entry(position_actual_value, slave_pos,
                                        {0x6064, 0});  // position actual value

            // velocity
            task.try_register_pdo_entry(target_velocity, slave_pos,
                                        {0x60ff, 0});  // target velocity
            task.try_register_pdo_entry(velocity_actual_value, slave_pos,
                                        {0x606c, 0});  // velocity actual value

            // torque
            task.try_register_pdo_entry(target_torque, slave_pos,
                                        {0x6071, 0});  // target torque
            task.try_register_pdo_entry(torque_actual_value, slave_pos,
                                        {0x6077, 0});  // torque actual value
        });

    // 设置 activation 的回调函数, 将在 task.start 执行后调用 在 config 后 具体在主站激活后
    // task.set_activation_callback([&] { activation_callback(task); });

    // 设置 receive 的回调函数, 将在周期 receive 阶段后执行
    // task.set_receive_callback([&] { receive_callback(task); });

    // 设置 cycle 的回调函数, 将在周期 cycle 阶段时执行
    task.set_cycle_callback(
        [&]
        {
            if (cycle_count == 1000)
            {
                *control_word = 6;  // 切换 ds402 状态机
            }
            if (cycle_count == 2000)
            {
                *control_word = 7;  // 切换 ds402 状态机
                *mode_of_operation = 9;  // 切换 ds402 控制模式至周期同步速度模式
            }
            if (cycle_count >= 3000)
            {
                *control_word = 0xf;  // 切换 ds402 状态机
                *target_velocity = (1 << 17);  // 指定速度为 17bit 编码器的 1rpm 速度
            }
        });

    // 设置 send 的回调函数, 将在周期 send 阶段后执行
    task.set_send_callback(
        [&]
        {
            cycle_count++;
        });

    // clang-format on

    // 启动任务
    task.start();

    // 等待任务退出
    task.wait();

    // 资源释放
    task.release();

    return 0;
}
