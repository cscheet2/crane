#include "gamepad/gamepad_controller.hpp"

namespace gamepad
{

controller_interface::CallbackReturn GamepadPublisher::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joy_pub_->on_activate();
  RCLCPP_INFO(get_node()->get_logger(), "GamepadPublisher activated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GamepadPublisher::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joy_pub_->on_deactivate();
  RCLCPP_INFO(get_node()->get_logger(), "GamepadPublisher deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GamepadPublisher::on_init()
{
  joy_pub_ = get_node()->create_publisher<sensor_msgs::msg::Joy>("joy", 10);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GamepadPublisher::on_configure(const rclcpp_lifecycle::State &)
{
  axis_states_.clear();
  button_states_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "Configuring GamepadPublisher with %zu state interfaces", state_interfaces_.size());
  
  for (const auto & state : state_interfaces_)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Found interface %s/%s", state.get_name().c_str(), state.get_interface_name().c_str());
    if (state.get_interface_name() == "position")
    {
      if (state.get_name().rfind("axis_", 0) == 0)
      {
        axis_states_.push_back(std::cref(state));
        RCLCPP_INFO(get_node()->get_logger(), "Added axis interface: %s", state.get_name().c_str());

      }
      else if (state.get_name().rfind("button_", 0) == 0)
      {
        button_states_.push_back(std::cref(state));
        RCLCPP_INFO(get_node()->get_logger(), "Added button interface: %s", state.get_name().c_str());
      }
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Configured with %zu axes and %zu buttons", axis_states_.size(), button_states_.size());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GamepadPublisher::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration GamepadPublisher::state_interface_configuration() const
{
  std::vector<std::string> interface_names;
  
  // Add the axes you want to subscribe to
  interface_names.push_back("axis_0/position");
  interface_names.push_back("axis_1/position");
  interface_names.push_back("axis_2/position");
  interface_names.push_back("axis_3/position");
  
  // Add the buttons you want to subscribe to
  interface_names.push_back("button_0/position");
  interface_names.push_back("button_1/position");
  interface_names.push_back("button_2/position");
  interface_names.push_back("button_3/position");
  
  return {
    controller_interface::interface_configuration_type::INDIVIDUAL, interface_names};
}

controller_interface::return_type GamepadPublisher::update(const rclcpp::Time &, const rclcpp::Duration &)
{
  sensor_msgs::msg::Joy joy_msg;
  joy_msg.header.stamp = get_node()->get_clock()->now();
  joy_msg.axes.reserve(axis_states_.size());
  joy_msg.buttons.reserve(button_states_.size());
  
  for (const auto & axis : axis_states_)
  {
    auto value = axis.get().get_optional();
    // joy_msg.axes.push_back(static_cast<float>(value.value_or(0.0)));
    if (value.has_value())
    {
      joy_msg.axes.push_back(static_cast<float>(value.value()));
      RCLCPP_INFO(get_node()->get_logger(), "Axis %s: %f", axis.get().get_name().c_str(), value.value());
    } 
    else {
      joy_msg.axes.push_back(0.0f);
      RCLCPP_WARN(get_node()->get_logger(), "Axis %s has no value", axis.get().get_name().c_str());
    }
  }
    
  for (const auto & button : button_states_)
  {
    auto value = button.get().get_optional();
    // joy_msg.buttons.push_back(static_cast<int32_t>(value.value_or(0.0)));
    if (value.has_value())
    {
      joy_msg.buttons.push_back(static_cast<int32_t>(value.value()));
      RCLCPP_INFO(get_node()->get_logger(), "Button %s: %f", button.get().get_name().c_str(), value.value());
    }
    else {
      joy_msg.buttons.push_back(0);
      RCLCPP_WARN(get_node()->get_logger(), "Button %s has no value", button.get().get_name().c_str());
    }
  }
  RCLCPP_INFO(get_node()->get_logger(), "Publishing Joy msg with %zu axes and %zu buttons", joy_msg.axes.size(), joy_msg.buttons.size());
  joy_pub_->publish(joy_msg);
  return controller_interface::return_type::OK;
}

} // namespace gamepad

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gamepad::GamepadPublisher, controller_interface::ControllerInterface)