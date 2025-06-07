#include "gamepad/gamepad_controller.hpp"

namespace gamepad
{
controller_interface::CallbackReturn GamepadPublisher::on_init()
{
  joy_pub_ = get_node()->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GamepadPublisher::on_configure(const rclcpp_lifecycle::State &)
{
  axis_states_.clear();
  button_states_.clear();

  for (const auto & state : state_interfaces_) 
  {
    if (state.get_interface_name() == "position")
    {
      if (state.get_name().rfind("axis_", 0) == 0)
        axis_states_.push_back(std::cref(state));
      else if (state.get_name().rfind("button_", 0) == 0)
        button_states_.push_back(std::cref(state));
    }
  }
}

controller_interface::InterfaceConfiguration GamepadPublisher::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration GamepadPublisher::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {}};
}

controller_interface::return_type GamepadPublisher::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  sensor_msgs::msg::Joy joy_msg;
  joy_msg.axes.reserve(axis_states_.size());
  joy_msg.buttons.reserve(button_states_.size());

  for (const auto & axis : axis_states_)
    joy_msg.axes.push_back(axis.get().get_value());
  
  for (const auto & button : button_states_) 
    joy_msg.buttons.push_back(button.get().get_value());

  joy_pub_->publish(joy_msg);
  return controller_interface::return_type::OK;
}

}  // namespace gamepad