
#include "motor_comm/motorComm.h"
#include "motor_comm/IPCComm.h"
#include <signal.h>
#include <memory>
#include <unistd.h>
// 必须引入：多线程执行器头文件
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <chrono>

// 全局智能指针（信号处理器中可访问IPCComm实例）
std::shared_ptr<IPCComm> g_ipc;
// 退出标志（原子变量保证线程安全）
std::atomic<bool> g_exit_flag(false);

// 自定义SIGINT信号处理器：接管Ctrl+C，执行优雅退出逻辑
void signal_handler(int signum) {
    if (signum == SIGINT && !g_exit_flag.load(std::memory_order_acquire)) {
        printf("\n[Signal] 收到Ctrl+C退出信号(SIGINT)，开始优雅退出流程...\n");
        
        // 1. 标记IPC通信线程停止
        if (g_ipc) {
            g_ipc->stop(); // 内部已包含：发送清零指令+关闭fd+回收线程
        }
        printf("[Signal] 电机清零指令已发送，通信资源已释放\n");
        g_exit_flag.store(true, std::memory_order_release);
    }
}

int main(int argc, char** argv){
    // 第一步：注册SIGINT信号处理器，接管Ctrl+C
    for (int i=0;i<23;i++)
    {
        if(i<12) {// 下肢
            global_Joint_data->kp[Motor_Index_Switch[i]]=100;    
            global_Joint_data->kd[Motor_Index_Switch[i]]=3.0;
        }else { // 上肢
            global_Joint_data->kp[Motor_Index_Switch[i]]=30;    
            global_Joint_data->kd[Motor_Index_Switch[i]]=2.0;
        }
          
        global_Joint_data->position[Motor_Index_Switch[i]]=0.0;  
        global_Joint_data->velocity[Motor_Index_Switch[i]]=0.0; 
        global_Joint_data->effort[Motor_Index_Switch[i]]=0.0; 
    }
    global_Joint_data->kp[Motor_Index_Switch[12]] = 1;
    global_Joint_data->kd[Motor_Index_Switch[12]] = 0.1;

    if (signal(SIGINT, signal_handler) == SIG_ERR) {
        perror("注册信号处理器失败");
        return -1;
    }
    // 第二步：初始化ROS2和IPC通信
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorComm>("motor_comm_node");
    g_ipc = std::make_shared<IPCComm>(); // 初始化后自动启动通信线程
    printf("[Main] ROS2节点和IPC通信已初始化，程序正常运行（按Ctrl+C优雅退出）\n");

    // 第三步：创建ROS2多线程执行器（核心修复，解决调度阻塞）
    rclcpp::executors::MultiThreadedExecutor executor;
    // 将节点添加到执行器，执行器会后台管理所有回调（定时器/订阅）
    executor.add_node(node);

    // 第四步：后台调度回调 + 主线程检测退出标志（无阻塞、无延时）
    // 执行器后台轮询（独立线程），主线程仅循环检测退出标志，无任何睡眠/阻塞
    while (!g_exit_flag.load(std::memory_order_acquire) && rclcpp::ok()) {
        // 非阻塞执行：处理所有待调度的回调，立即返回（无回调时不占用资源）
        executor.spin_once(std::chrono::microseconds(10));
    }

    // 第五步：优雅收尾（双重保障，释放所有资源）
    executor.remove_node(node); // 从执行器移除节点，停止回调调度
    if (g_ipc) {
        g_ipc.reset(); // 销毁IPCComm实例，调用析构函数兜底释放fd/线程
    }
    rclcpp::shutdown(); // 关闭ROS2核心，释放所有底层资源
    printf("[Main] 程序已优雅退出，所有资源释放完成\n");

    return 0;
}
