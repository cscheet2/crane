#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motor_speed.hpp"

class LeftMotor : public rclcpp::Node {
  public:
    LeftMotor() : Node("leftmotor") {
      subscription_ = this->create_subscription<interfaces::msg::MotorSpeed>("leftmotor", 10, 
        [this](interfaces::msg::MotorSpeed::UniquePtr msg) -> void {
          RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->speed);
        }
      );
    }
  private:
    rclcpp::Subscription<interfaces::msg::MotorSpeed>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeftMotor>());
  rclcpp::shutdown();
  return 0;
}