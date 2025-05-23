#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * Node to initalize the robots joystick control
 */
class ControlPublisher : public rclcpp::Node {
  public:
    ControlPublisher() : Node("control_publisher"), count_(0) { 
      publisher_ = this->create_publisher<std::message::msg::String>("topic", 10);
      auto timer_callback = [this]() -> void {
        auto message = std_msgs::msg::String();
        message.data = ":3 " + std::to_string(this->count_++);
      }
      
    }
}