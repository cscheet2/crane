#ifndef _GAMEPAD_CONTROLLER_HPP
#define _GAMEPAD_CONTROLLER_HPP

#include "controller_interface/controller_interface.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace gamepad
{

class GamepadPublisher : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
};

} // namespace gamepad_controller

#endif // _GAMEPAD_CONTROLLER_HPP