#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/excavation.hpp"

class Excavation : public rclcpp::Node {
  public:
    Excavation() : Node("drive_train") {
      auto topic_callback = [this](interfaces::msg::Excavation::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
      };
      subscription_ = this->create_subscription<interfaces::msg::Excavation>("excavation", 10, topic_callback);
    }
  private:
    rclcpp::Subscription<interfaces::msg::Excavation>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Excavation>());
  rclcpp::shutdown();
  return 0;
}