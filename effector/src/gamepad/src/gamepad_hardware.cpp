#include <cmath>
#include "gamepad/gamepad_hardware.hpp"

namespace gamepad
{

hardware_interface::CallbackReturn GamepadHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("device") == info_.hardware_parameters.end()) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("GamepadHardware"), "Device parameter not found in URDF");
    return CallbackReturn::ERROR;
  }

  device_path_ = info_.hardware_parameters.at("device");
  if (access(device_path_.c_str(), F_OK) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("GamepadHardware"), "Device does not exist: %s", device_path_.c_str());
    return CallbackReturn::ERROR;
  }

  // allocate space for axes/buttons
  axes_.resize(4, 0.0);
  buttons_.resize(4, 0.0);

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GamepadHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("GamepadHardware"), "Configured gamepad hardware");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GamepadHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);
  if (fd_ < 0) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("GamepadHardware"), "Failed to open device: %s", device_path_.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("GamepadHardware"), "Activated gamepad hardware");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GamepadHardware::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("GamepadHardware"), "Deactivated gamepad hardware");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GamepadHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < axes_.size(); ++i)
  {
    state_interfaces.emplace_back("axis_" + std::to_string(i), "position", &axes_[i]);
  }
  
  for (size_t i = 0; i < buttons_.size(); ++i)
  {
    state_interfaces.emplace_back("button_" + std::to_string(i), "position", &buttons_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GamepadHardware::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // Gamepad is an input device, so no command interfaces needed
    return command_interfaces;
}

hardware_interface::return_type GamepadHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{ 
  if (fd_ < 0) 
  {
    return hardware_interface::return_type::ERROR;
  }
  struct js_event event;
  while (::read(fd_, &event, sizeof(event)) > 0) {
    event.type &= ~JS_EVENT_INIT;  // Ignore init events

    if (event.type == JS_EVENT_AXIS && event.number < axes_.size())
    {
      axes_[event.number] = static_cast<double>(event.value) / 32767.0;
      RCLCPP_INFO(rclcpp::get_logger("GamepadHardware"), "Axis %d: %f", event.number, axes_[event.number]);
    }
    else if (event.type == JS_EVENT_BUTTON && event.number < buttons_.size())
    {
      buttons_[event.number] = static_cast<double>(event.value);
      RCLCPP_INFO(rclcpp::get_logger("GamepadHardware"), "Button %d: %f", event.number, buttons_[event.number]);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GamepadHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Gamepad is an input device, so there is no need to write
  return hardware_interface::return_type::OK;
}

}  // namespace gamepad

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(gamepad::GamepadHardware, hardware_interface::SystemInterface)