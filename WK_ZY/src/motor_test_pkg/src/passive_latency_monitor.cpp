#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "motor_interface/msg/joint_motor.hpp"
#include "probe/single_shot_probe.h"

namespace
{
constexpr int kJointCount = 23;
constexpr std::array<int, kJointCount> kMotorIndexSwitch = {
    0, 1, 2, 3, 4, 5,
    6, 7, 8, 9, 10, 11,
    14, 16, 18, 17, 15,
    20, 22, 21, 19, 12, 13};

bool derive_master_motor(int ros_joint_index, int &master_index, int &motor_index)
{
    for (int wire_index = 0; wire_index < kJointCount; ++wire_index)
    {
        if (kMotorIndexSwitch[wire_index] != ros_joint_index)
        {
            continue;
        }

        if (wire_index < 12)
        {
            master_index = 0;
            motor_index = wire_index;
        }
        else if (wire_index < 17)
        {
            master_index = 1;
            motor_index = wire_index - 12;
        }
        else
        {
            master_index = 2;
            motor_index = wire_index - 17;
        }
        return true;
    }
    return false;
}

std::int64_t stamp_to_ns(const builtin_interfaces::msg::Time &stamp)
{
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
           static_cast<std::int64_t>(stamp.nanosec);
}
} // namespace

class PassiveLatencyMonitor : public rclcpp::Node
{
public:
    PassiveLatencyMonitor()
        : Node("motor_latency_monitor_node"), trace_(single_shot_probe::map_trace())
    {
        target_ros_joint_index_ = this->declare_parameter<int>("target_ros_joint_index", 0);
        target_master_index_ = this->declare_parameter<int>("target_master_index", -1);
        target_motor_index_ = this->declare_parameter<int>("target_motor_index", -1);
        oneshot_ = this->declare_parameter<bool>("oneshot", true);
        use_header_stamp_ = this->declare_parameter<bool>("use_header_stamp", true);

        if (target_master_index_ < 0 || target_motor_index_ < 0)
        {
            int derived_master = -1;
            int derived_motor = -1;
            if (derive_master_motor(target_ros_joint_index_, derived_master, derived_motor))
            {
                target_master_index_ = derived_master;
                target_motor_index_ = derived_motor;
            }
        }
        if (target_master_index_ < 0 || target_motor_index_ < 0)
        {
            RCLCPP_WARN(this->get_logger(),
                        "failed to derive EtherCAT target from ros joint %d; set target_master_index and target_motor_index explicitly",
                        target_ros_joint_index_);
        }

        if (trace_ == nullptr)
        {
            RCLCPP_FATAL(this->get_logger(), "failed to map single-shot probe shared memory");
            return;
        }

        arm_next_command();

        auto qos = rclcpp::QoS(10).reliable();
        command_sub_ = this->create_subscription<motor_interface::msg::JointMotor>(
            "/joint_command",
            qos,
            std::bind(&PassiveLatencyMonitor::command_callback, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<motor_interface::msg::JointMotor>(
            "/joint_states_comm",
            qos,
            std::bind(&PassiveLatencyMonitor::state_callback, this, std::placeholders::_1));
    }

private:
    void arm_next_command()
    {
        if (trace_ == nullptr)
        {
            return;
        }
        command_seen_ns_ = 0U;
        command_header_ns_ = 0;
        command_position_ = 0.0;
        single_shot_probe::arm_passive_trace(
            trace_, target_ros_joint_index_, target_master_index_, target_motor_index_);
        RCLCPP_INFO(this->get_logger(),
                    "[PassiveLatency] armed next /joint_command: ros_joint=%d master=%d motor=%d",
                    target_ros_joint_index_,
                    target_master_index_,
                    target_motor_index_);
    }

    void command_callback(const motor_interface::msg::JointMotor::SharedPtr msg)
    {
        if (trace_ == nullptr || !single_shot_probe::ready(trace_) || command_seen_ns_ != 0U)
        {
            return;
        }
        if (target_ros_joint_index_ < 0 || target_ros_joint_index_ >= kJointCount)
        {
            return;
        }

        const double position = msg->position[target_ros_joint_index_];
        if (!single_shot_probe::matches_command_position(trace_, position))
        {
            return;
        }

        command_seen_ns_ = single_shot_probe::now_ns();
        command_header_ns_ = stamp_to_ns(msg->header.stamp);
        command_position_ = position;
        if (single_shot_probe::command_position_is_any(trace_))
        {
            trace_->command_position_rad = position;
        }

        RCLCPP_INFO(this->get_logger(),
                    "[PassiveLatency] observed /joint_command: command_seen_ns=%llu joint=%d pos=%.6f header_ns=%lld",
                    static_cast<unsigned long long>(command_seen_ns_),
                    target_ros_joint_index_,
                    command_position_,
                    static_cast<long long>(command_header_ns_));
    }

    void state_callback(const motor_interface::msg::JointMotor::SharedPtr msg)
    {
        if (trace_ == nullptr || trace_->summary_printed != 0U ||
            trace_->state_pub_ns == 0U || trace_->state_seen_ns != 0U)
        {
            return;
        }
        if (target_ros_joint_index_ < 0 || target_ros_joint_index_ >= kJointCount)
        {
            return;
        }

        const std::int64_t header_ns = stamp_to_ns(msg->header.stamp);
        if (header_ns != trace_->state_pub_header_ns ||
            std::fabs(msg->position[target_ros_joint_index_] - trace_->state_pub_position) > 1e-6 ||
            std::fabs(msg->velocity[target_ros_joint_index_] - trace_->state_pub_velocity) > 1e-6)
        {
            return;
        }

        trace_->state_seen_ns = single_shot_probe::now_ns();
        trace_->state_seen_position = msg->position[target_ros_joint_index_];
        trace_->state_seen_velocity = msg->velocity[target_ros_joint_index_];

        if (command_seen_ns_ != 0U)
        {
            RCLCPP_INFO(this->get_logger(),
                        "[PassiveLatency] observer /joint_command -> observer /joint_states_comm = %.3f us",
                        single_shot_probe::delta_us(command_seen_ns_, trace_->state_seen_ns));
        }

        if (use_header_stamp_ && command_header_ns_ > 0)
        {
            const std::int64_t state_seen_ros_ns = this->get_clock()->now().nanoseconds();
            const double header_to_state_seen_us =
                static_cast<double>(state_seen_ros_ns - command_header_ns_) / 1000.0;
            RCLCPP_INFO(this->get_logger(),
                        "[PassiveLatency] /joint_command header.stamp -> observer /joint_states_comm = %.3f us",
                        header_to_state_seen_us);
        }

        single_shot_probe::print_summary_if_ready(trace_);

        if (!oneshot_)
        {
            arm_next_command();
        }
    }

    single_shot_probe::TraceData *trace_ = nullptr;
    int target_ros_joint_index_ = 0;
    int target_master_index_ = 0;
    int target_motor_index_ = 0;
    bool oneshot_ = true;
    bool use_header_stamp_ = true;
    std::uint64_t command_seen_ns_ = 0U;
    std::int64_t command_header_ns_ = 0;
    double command_position_ = 0.0;
    rclcpp::Subscription<motor_interface::msg::JointMotor>::SharedPtr command_sub_;
    rclcpp::Subscription<motor_interface::msg::JointMotor>::SharedPtr state_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PassiveLatencyMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
