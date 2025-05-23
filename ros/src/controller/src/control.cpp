#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/drive_train.hpp"
#include "interfaces/msg/excavation.hpp"
#include "interfaces/msg/hopper.hpp"
#include "interfaces/msg/vision.hpp"

using namespace rclcpp;
using namespace std::chrono_literals;

/**
 * Node to initalize the robots joystick control
 */
class Controller : public Node {
  public:
    Controller() : Node("controller"), count_(0) { 
      drivetrain_publisher_ = this->create_publisher<interfaces::msg::DriveTrain>("drive-train", 10);
      auto timer_callback = [this]() -> void {
        auto message = interfaces::msg::DriveTrain();
        message.data = ":3 " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "publishing '%s'", message.data.c_str());
        this->drivetrain_publisher_->publish(message);
      };
      timer_ = this->create_wall_timer(500ms, timer_callback);
    }
  private:
    TimerBase::SharedPtr timer_;
    Publisher<interfaces::msg::DriveTrain>::SharedPtr drivetrain_publisher_;
    // Publisher<interfaces::msg::Excavation>::SharedPtr execavation_publisher_;
    
    size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}