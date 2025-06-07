#include "remote_control/remote_control.hpp"

namespace remote_control 
{

hardware_interface::CallbackReturn RemoteControl::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RemoteControl::export_state_interfaces()
{
  return {
    hardware_interface::StateInterface("joystick_axis_x", hardware_interface::HW_IF_POSITION, &axis_x_),
    hardware_interface::StateInterface("joystick_axis_y", hardware_interface::HW_IF_POSITION, &axis_y_),
  };
}

hardware_interface::return_type RemoteControl::read(const rclcpp::Time &, const rclcpp::Duration &)
{ 
  // TODO: replace with real controller input
  axis_x_ =  0.1;
  axis_y_ = -0.1;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type remote_control::RemoteControl::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // TODO: Implement controller output if needed.
  return hardware_interface::return_type::OK;
}

}  // namespace remote_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(remote_control::RemoteControl, hardware_interface::SystemInterface)