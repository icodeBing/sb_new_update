#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("axis_data", 10);
    timer_ = this->create_wall_timer(
        1ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback() {
#define MAX_VELOCITY (1 << 17)                   // 定义最大速度
#define VELOCITY_INCREMENT (MAX_VELOCITY / 1000) // 定义加减速度增量

    // Velocity phase
    static bool velocity_phase = true; // 加速和减速
    // Direction
    static bool direction = true; // 速度是否为正

    if (velocity_phase) {
      if (direction) {
        if (publishing_velocity < MAX_VELOCITY) {
          publishing_velocity += VELOCITY_INCREMENT;
        } else {
          velocity_phase = false; // 达到最大值，开始减少
        }
      } else {
        if (publishing_velocity > -MAX_VELOCITY) {
          publishing_velocity -= VELOCITY_INCREMENT;
        } else {
          velocity_phase = false; // 达到最小值，开始增加
        }
      }
    } else {
      if (direction) {
        if (publishing_velocity > 0) {
          publishing_velocity -= VELOCITY_INCREMENT;
        } else {
          direction = false;     // 速度变为负
          velocity_phase = true; // 开始增加
        }
      } else {
        if (publishing_velocity < 0) {
          publishing_velocity += VELOCITY_INCREMENT;
        } else {
          direction = true;      // 速度变为正
          velocity_phase = true; // 开始增加
        }
      }
    }

    // 创建消息并发布
    auto message = std_msgs::msg::String();
    message.data = std::to_string(publishing_velocity);
    if (count_ % 1000 == 0) {
      RCLCPP_INFO(this->get_logger(), "Publishing axis velocity: %s",
                  message.data.c_str());
      // printf("Publishing: '%s'\n", message.data.c_str());
    }
    count_++;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  int32_t publishing_velocity;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
