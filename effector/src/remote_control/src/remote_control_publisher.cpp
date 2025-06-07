#include "remote_control/remote_control_publisher.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace remote_control
{

controller_interface::InterfaceConfiguration RemoteControlPublisher::command_interface_configuration() const 
{
  return {
    controller_interface::interface_configuration_type::NONE, 
    {},
  };
}

controller_interface::InterfaceConfiguration RemoteControlPublisher::state_interface_configuration() const
{
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL,
    {"joystick_axis_x", "joystick_axis_y"},
  };
}

controller_interface::CallbackReturn RemoteControlPublisher::on_init() 
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RemoteControlPublisher::on_configure(const rclcpp_lifecycle::State &previous_state)
{
  controller_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Joy>("~/joy", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RemoteControlPublisher::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  // axis_x_ = state_interfaces_[0];
  // axis_y_ = state_interfaces_[1];
  // axis_x_ptr_ = state_interfaces_[0].;
  state_interfaces_[0];
  RCLCPP_INFO(rclcpp::get_logger("RemoteControlPublisher"), state_interfaces_[0].get_full_name().c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn RemoteControlPublisher::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  controller_publisher_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type RemoteControlPublisher::update(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  sensor_msgs::msg::Joy msg;
  msg.header.stamp = get_node()->now();
  // msg.axes = {axis_x_.get_value(), axis_y_.get_value()};
  controller_publisher_->publish(msg);
  return controller_interface::return_type::OK;
}

}  // namespace remote_controller

PLUGINLIB_EXPORT_CLASS(remote_control::RemoteControlPublisher, controller_interface::ControllerInterface)