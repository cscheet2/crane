#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/drive_train.hpp"


class DriveTrain : public rclcpp::Node {
  public:
    DriveTrain() : Node("drive_train") {
      auto topic_callback = [this](interfaces::msg::DriveTrain::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->message.c_str());
      };
      subscription_ = this->create_subscription<interfaces::msg::DriveTrain>("topic", 10, topic_callback);
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