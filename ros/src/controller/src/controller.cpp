#include "controller/controller.hpp"

Controller::Controller() : Node("controller"), count_(0) 
{
  drivetrain_publisher_ = this->create_publisher<interfaces::msg::DriveTrain>("drivetrain", 10);

  auto timer_callback = [this]() -> void {
    auto message = interfaces::msg::DriveTrain();
    message.data = ":3 " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "publishing '%s'", message.data.c_str());
    this->drivetrain_publisher_->publish(message);
  };

  timer_ = this->create_wall_timer(500ms, timer_callback);
}

int main(int argc, char* argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}
