#include "motor_comm/motorComm.h"

#include <cmath>

#include "probe/single_shot_probe.h"

MotorComm::MotorComm(const std::string& node_name): Node(node_name){
    init();
}

MotorComm::~MotorComm(){

}

void MotorComm::init(){
    rclcpp::QoS qos = rclcpp::QoS(10).reliable().transient_local();//设置 QoS 为可靠 + 瞬态本地 + 深度 10，确保消息不丢失、实时传输
    pub_ = this->create_publisher<JointMotor>("/joint_states_comm", qos);
    if (pub_) {
        RCLCPP_INFO(this->get_logger(), "发布者创建成功（话题：joint_states_comm）");
    } else {
        RCLCPP_FATAL(this->get_logger(), "发布者创建失败！");
    }

    sub_ = this->create_subscription<JointMotor>("/joint_command", qos,
        std::bind(&MotorComm::joint_motor_comm_callback,this, std::placeholders::_1));
    if (sub_) {
        RCLCPP_INFO(this->get_logger(), "订阅者创建成功（话题：joint_command）");
    } else {
        RCLCPP_FATAL(this->get_logger(), "订阅者创建失败！");
    }

    timer_ = this->create_wall_timer(5ms, std::bind(&MotorComm::timer_callback, this));
    if (timer_) {
        RCLCPP_INFO(this->get_logger(), "定时器创建成功（周期：5ms）");
    } else {
        RCLCPP_FATAL(this->get_logger(), "定时器创建失败！");
    }
}

void MotorComm::timer_callback(){
    std::lock_guard<std::mutex> lock(motor_mutex);
    if (global_Joint_data_pub == nullptr){
        RCLCPP_WARN(this -> get_logger(), "global_Joint_data_pub is nullptr");
        return;
    }
    auto msg = JointMotor(); 
    // 设置时间戳和坐标系
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "base_link";    
    // 设置关节数据
    msg.joint_names = joint_names;
    msg.position = global_Joint_data_pub->position;
    msg.velocity = global_Joint_data_pub->velocity;
    msg.effort = global_Joint_data_pub->effort;
    // 发布消息
    pub_->publish(msg);
    static auto *trace = single_shot_probe::map_trace();
    if (trace != nullptr && single_shot_probe::ready(trace) &&
        trace->ipc_feedback_rx_ns != 0U && trace->state_pub_ns == 0U)
    {
        trace->state_pub_ns = single_shot_probe::now_ns();
        const int joint_index = trace->target_ros_joint_index;
        trace->state_pub_header_ns =
            static_cast<std::int64_t>(msg.header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(msg.header.stamp.nanosec);
        if (joint_index >= 0 && joint_index < motor_num)
        {
            trace->state_pub_position = msg.position[joint_index];
            trace->state_pub_velocity = msg.velocity[joint_index];
        }
        RCLCPP_INFO(this->get_logger(),
                    "[Probe][MotorComm] 首次发布 /joint_states_comm，state_pub_ns=%llu joint=%d pos=%.6f vel=%.6f",
                    static_cast<unsigned long long>(trace->state_pub_ns),
                    joint_index,
                    trace->state_pub_position,
                    trace->state_pub_velocity);
    }
    // RCLCPP_INFO(this->get_logger(), "ROS2节点timer_callback正常运行\n");
   }

void MotorComm::joint_motor_comm_callback(const JointMotor::SharedPtr msg){
    std::lock_guard<std::mutex> lock(joint_mutex);
    static auto *trace = single_shot_probe::map_trace();
    if (trace != nullptr && single_shot_probe::ready(trace) && trace->motor_node_rx_ns == 0U)
    {
        const int joint_index = trace->target_ros_joint_index;
        if (joint_index >= 0 && joint_index < motor_num &&
            std::fabs(msg->position[joint_index] - trace->command_position_rad) < 1e-9)
        {
            trace->motor_node_rx_ns = single_shot_probe::now_ns();
            RCLCPP_INFO(this->get_logger(),
                        "[Probe][MotorComm] 收到 /joint_command，motor_node_rx_ns=%llu joint=%d pos=%.6f",
                        static_cast<unsigned long long>(trace->motor_node_rx_ns),
                        joint_index,
                        msg->position[joint_index]);
        }
    }
    global_Joint_data = msg;
    // RCLCPP_INFO(this->get_logger(), "ROS2节点joint_motor_comm_callback正常运行\n");
    // RCLCPP_INFO(this->get_logger(), "\n接收关节状态：");
    // for (size_t i = 0; i < motor_num; ++i) {
    //     RCLCPP_INFO(this->get_logger(), "  %s: kp= %.2f, kd= %.2f, 位置=%.2f, 速度=%.2f, 力矩=%.2f",
    //                 msg->joint_names[i].c_str(), 
    //                 msg->kp[i],
    //                 msg->kd[i],
    //                 msg->position[i], 
    //                 msg->velocity[i], 
    //                 msg->effort[i]);
    // }
    // RCLCPP_INFO(this->get_logger(), "-------------joint_motor_comm_callback--------------");
}
