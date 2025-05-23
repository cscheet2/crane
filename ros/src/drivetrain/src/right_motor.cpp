#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/motor_speed.hpp"

class RightMotor : public rclcpp::Node {
  public:
    RightMotor() : Node("drivetrain") {
      auto topic_callback = [this](interfaces::msg::MotorSpeed::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->speed);
      };
      subscription_ = this->create_subscription<interfaces::msg::MotorSpeed>("rightmotor", 10, topic_callback);
    }
  private:
    rclcpp::Subscription<interfaces::msg::MotorSpeed>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RightMotor>());
  rclcpp::shutdown();
  return 0;
}