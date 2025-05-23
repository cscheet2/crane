#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/drive_train.hpp"

class DriveTrain : public rclcpp::Node {
  public:
    DriveTrain() : Node("drivetrain") {
      auto topic_callback = [this](interfaces::msg::DriveTrain::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
      subscription_ = this->create_subscription<interfaces::msg::DriveTrain>("drivetrain", 10, topic_callback);
    }
  private:
    rclcpp::Subscription<interfaces::msg::DriveTrain>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveTrain>());
  rclcpp::shutdown();
  return 0;
}