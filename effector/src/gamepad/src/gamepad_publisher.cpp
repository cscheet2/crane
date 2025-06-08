#include "gamepad/gamepad_publisher.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gamepad
{

controller_interface::CallbackReturn GamepadPublisher::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GamepadPublisher::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration GamepadPublisher::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  
  // Add all axis and button interfaces
  for (int i = 0; i < 4; ++i) {
    config.names.push_back("axis_" + std::to_string(i) + "/position");
  }
  for (int i = 0; i < 4; ++i) {
    config.names.push_back("button_" + std::to_string(i) + "/position");
  }
  
  return config;
}

controller_interface::CallbackReturn GamepadPublisher::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create publisher
  joy_publisher_ = get_node()->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
  
  RCLCPP_INFO(get_node()->get_logger(), "Gamepad controller configured");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GamepadPublisher::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Gamepad controller activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GamepadPublisher::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Gamepad controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GamepadPublisher::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Create Joy message
  auto joy_msg = std::make_unique<sensor_msgs::msg::Joy>();
  joy_msg->header.stamp = time;
  joy_msg->header.frame_id = "gamepad";
  
  // Read axes
  joy_msg->axes.resize(4);
  for (int i = 0; i < 4; ++i) {
    joy_msg->axes[i] = state_interfaces_[i].get_value();
  }
  
  // Read buttons
  joy_msg->buttons.resize(4);
  for (int i = 0; i < 4; ++i) {
    joy_msg->buttons[i] = static_cast<int32_t>(state_interfaces_[i + 4].get_value());
  }
  
  // Publish the message
  joy_publisher_->publish(std::move(joy_msg));
  
  return controller_interface::return_type::OK;
}

} // namespace gamepad_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gamepad::GamepadPublisher, controller_interface::ControllerInterface)