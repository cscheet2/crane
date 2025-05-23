#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/vision.hpp"

class Vision : public rclcpp::Node {
  public:
    Vision() : Node("drive_train") {
      auto topic_callback = [this](interfaces::msg::Vision::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
      subscription_ = this->create_subscription<interfaces::msg::Vision>("drive-train", 10, topic_callback);
    }
  private:
    rclcpp::Subscription<interfaces::msg::Vision>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Vision>());
  rclcpp::shutdown();
  return 0;
}