#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/hopper.hpp"

class Hopper : public rclcpp::Node {
  public:
    Hopper() : Node("drive_train") {
      auto topic_callback = [this](interfaces::msg::Hopper::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
      subscription_ = this->create_subscription<interfaces::msg::Hopper>("hopper", 10, topic_callback);
    }
  private:
    rclcpp::Subscription<interfaces::msg::Hopper>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Hopper>());
  rclcpp::shutdown();
  return 0;
}