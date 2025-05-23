#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/drive_train.hpp"

using namespace std::chrono_literals;

/**
 * Node to initalize the robots joystick control
 */
class Controller : public rclcpp::Node {
  public:
    Controller() : Node("controller"), count_(0) { 
      publisher_ = this->create_publisher<interfaces::msg::DriveTrain>("topic", 10);
      auto timer_callback = [this]() -> void {
        auto message = interfaces::msg::DriveTrain();
        message.message = ":3 " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "publishing '%s'", message.message.c_str());
        this->publisher_->publish(message);
      };
      timer_ = this->create_wall_timer(500ms, timer_callback);
    }
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::DriveTrain>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}