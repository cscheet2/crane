
#ifndef GAMEPAD_PUBLISHER_HPP
#define GAMEPAD_PUBLISHER_HPP

#include <controller_interface/controller_interface.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

namespace gamepad
{
class GamepadPublisher : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>> joy_pub_;
  std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> axis_states_;
  std::vector<std::reference_wrapper<const hardware_interface::LoanedStateInterface>> button_states_;
};
} // namespace gamepad

#endif // GAMEPAD_PUBLISHER_HPP
