#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "motor_interface/msg/joint_motor.hpp"
#include "probe/single_shot_probe.h"

using namespace std::chrono_literals;

namespace
{
    constexpr int kTargetRosJointIndex = 0;
    constexpr int kTargetMasterIndex = 0;
    constexpr int kTargetMotorIndex = 0;
    constexpr double kTargetPositionRad = -0.2;
    constexpr double kTargetKp = 200.0;
    constexpr double kTargetKd = 2.0;

    const std::array<std::string, 23> kJointNames = {
        "neck_ti5_joint1", "arm_L1_ti5_joint2", "arm_L2_ti5_joint3",
        "arm_L3_ti5_joint4", "arm_L4_ti5_joint5", "waist_ti5_1_joint6",
        "waist_ti5_2_joint7", "arm_R1_ti5_joint8", "arm_R2_ti5_joint9",
        "arm_R3_ti5_joint10", "arm_R4_ti5_joint11", "leg_L1_enc_joint12",
        "leg_L2_enc_joint13", "leg_L3_enc_joint14", "leg_L4_enc_joint15",
        "leg_L5_enc_joint16", "leg_L6_enc_joint17", "leg_R1_enc_joint18",
        "leg_R2_enc_joint19", "leg_R3_enc_joint20", "leg_R4_enc_joint21",
        "leg_R5_enc_joint22", "leg_R6_enc_joint23"};
} // namespace

class MotorCommSingleShotProbe : public rclcpp::Node
{
public:
    explicit MotorCommSingleShotProbe(const std::string &node_name = "motor_comm_test_node")
        : Node(node_name), trace_(single_shot_probe::map_trace())
    {
        rclcpp::QoS qos = rclcpp::QoS(10).reliable().transient_local();
        data_pub_ = this->create_publisher<motor_interface::msg::JointMotor>("/joint_command", qos);
        data_sub_ = this->create_subscription<motor_interface::msg::JointMotor>(
            "/joint_states_comm", qos,
            std::bind(&MotorCommSingleShotProbe::joint_motor_data_callback, this, std::placeholders::_1));
        publish_timer_ = this->create_wall_timer(800ms, std::bind(&MotorCommSingleShotProbe::publish_once, this));

        init_joint_message();
        if (trace_ != nullptr)
        {
            single_shot_probe::arm_trace(
                trace_, kTargetRosJointIndex, kTargetMasterIndex, kTargetMotorIndex, kTargetPositionRad);
            RCLCPP_INFO(this->get_logger(),
                        "单步探针已布置，目标joint=%d master=%d motor=%d position=%.3f rad",
                        kTargetRosJointIndex,
                        kTargetMasterIndex,
                        kTargetMotorIndex,
                        kTargetPositionRad);
        }
    }

private:
    void init_joint_message()
    {
        data_.joint_names = kJointNames;
        std::fill(data_.kp.begin(), data_.kp.end(), 0.0);
        std::fill(data_.kd.begin(), data_.kd.end(), 0.0);
        std::fill(data_.position.begin(), data_.position.end(), 0.0);
        std::fill(data_.velocity.begin(), data_.velocity.end(), 0.0);
        std::fill(data_.effort.begin(), data_.effort.end(), 0.0);

        data_.kp[kTargetRosJointIndex] = kTargetKp;
        data_.kd[kTargetRosJointIndex] = kTargetKd;
        data_.position[kTargetRosJointIndex] = kTargetPositionRad;
    }

    void publish_once()
    {
        if (published_)
        {
            return;
        }

        data_.header.stamp = this->get_clock()->now();
        const std::uint64_t topic_pub_ns = single_shot_probe::now_ns();
        if (trace_ != nullptr && single_shot_probe::ready(trace_))
        {
            trace_->topic_pub_ns = topic_pub_ns;
        }
        data_pub_->publish(data_);
        published_ = true;
        publish_timer_->cancel();

        RCLCPP_INFO(this->get_logger(),
                    "[Probe][TopicTest] 已单发 /joint_command，topic_pub_ns=%llu target_joint=%d target_pos=%.6f",
                    static_cast<unsigned long long>(topic_pub_ns),
                    kTargetRosJointIndex,
                    data_.position[kTargetRosJointIndex]);
    }

    void joint_motor_data_callback(const motor_interface::msg::JointMotor::SharedPtr msg)
    {
        if (!published_ || trace_ == nullptr || trace_->state_pub_ns == 0U || trace_->state_seen_ns != 0U)
        {
            return;
        }
        const std::int64_t header_ns =
            static_cast<std::int64_t>(msg->header.stamp.sec) * 1000000000LL +
            static_cast<std::int64_t>(msg->header.stamp.nanosec);
        if (header_ns != trace_->state_pub_header_ns ||
            std::fabs(msg->position[kTargetRosJointIndex] - trace_->state_pub_position) > 1e-6 ||
            std::fabs(msg->velocity[kTargetRosJointIndex] - trace_->state_pub_velocity) > 1e-6)
        {
            return;
        }

        trace_->state_seen_ns = single_shot_probe::now_ns();
        trace_->state_seen_position = msg->position[kTargetRosJointIndex];
        trace_->state_seen_velocity = msg->velocity[kTargetRosJointIndex];

        RCLCPP_INFO(this->get_logger(),
                    "[Probe][TopicTest] 收到 /joint_states_comm，state_seen_ns=%llu joint=%d pos=%.6f vel=%.6f",
                    static_cast<unsigned long long>(trace_->state_seen_ns),
                    kTargetRosJointIndex,
                    trace_->state_seen_position,
                    trace_->state_seen_velocity);
        single_shot_probe::print_summary_if_ready(trace_);
    }

private:
    single_shot_probe::TraceData *trace_;
    bool published_ = false;
    rclcpp::Publisher<motor_interface::msg::JointMotor>::SharedPtr data_pub_;
    rclcpp::Subscription<motor_interface::msg::JointMotor>::SharedPtr data_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    motor_interface::msg::JointMotor data_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommSingleShotProbe>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
