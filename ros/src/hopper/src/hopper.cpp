#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "interfaces/msg/hopper.hpp"
#include "interfaces/action/move_linear_actuator.hpp"

class Hopper : public rclcpp::Node 
{
public:
  using MoveLinearActuator = interfaces::action::MoveLinearActuator; 
  Hopper() : Node("hopper") 
  {
    client_ = rclcpp_action::create_client<MoveLinearActuator>(this, "linear_actuator_action_client");

  }

private:
  rclcpp::Subscription<interfaces::msg::Hopper>::SharedPtr subscription_;
  rclcpp_action::Client<MoveLinearActuator>::SharedPtr client_;
};

int main(int argc, char* argv[]) 
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Hopper>());
  rclcpp::shutdown();
  return 0;
}














/**
     auto topic_callback = [this](interfaces::msg::Hopper::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    };
    subscription_ = this->create_subscription<interfaces::msg::Hopper>("hopper", 10, topic_callback);
 */