#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class DriveTrain : public rclcpp::Node {
  public:
    DriveTrain() : Node("drive_train") {
      auto topic_callback = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
    }
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveTrain>());
  rclcpp::shutdown();
  return 0;
}