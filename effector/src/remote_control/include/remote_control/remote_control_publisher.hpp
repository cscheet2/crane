#ifndef _REMOTE_CONTROL_PUBLISHER
#define _REMOTE_CONTROL_PUBLISHER

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace remote_control
{

class RemoteControlPublisher : public controller_interface::ControllerInterface
{
public:
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // hardware_interface::LoanedStateInterface axis_x_;
  // hardware_interface::LoanedStateInterface axis_y_;
  double axis_x_ = 0.0;
  double axis_y_ = 0.0;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr controller_publisher_;
};

}  // namespace remote_control

#endif  // _REMOTE_CONTROL_PUBLISHER