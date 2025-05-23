#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/drive_train.hpp"
#include "interfaces/msg/motor_speed.hpp"

class DriveTrain : public rclcpp::Node {
  public:
    DriveTrain() : Node("drivetrain") {
      left_motor_pub_  = this->create_publisher<interfaces::msg::MotorSpeed>("leftmotor",  10);
      right_motor_pub_ = this->create_publisher<interfaces::msg::MotorSpeed>("rightmotor", 10);
      subscription_ = this->create_subscription<interfaces::msg::DriveTrain>("drivetrain", 10, 
        [this](interfaces::msg::DriveTrain::UniquePtr msg) -> void {
          RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
        }
      );
    }
  private:
    rclcpp::Publisher<interfaces::msg::MotorSpeed>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<interfaces::msg::MotorSpeed>::SharedPtr right_motor_pub_;
    rclcpp::Subscription<interfaces::msg::DriveTrain>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveTrain>());
  rclcpp::shutdown();
  return 0;
}