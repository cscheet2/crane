#include <cmath>
#include "gamepad/gamepad_hardware.hpp"

namespace gamepad
{

hardware_interface::CallbackReturn GamepadHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  // store and open device path from config
  device_path_ = info_.hardware_parameters.at("device");
  fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK);

  if (fd_ < 0) 
  {
    RCLCPP_ERROR(rclcpp::get_logger("GamepadHardware"), "Failed to open device: %s", device_path_.c_str());
    return CallbackReturn::ERROR;
  }

  // allocate space for axes/buttons
  axes_.resize(4, 0.0);
  buttons_.resize(12, 0.0);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GamepadHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < axes_.size(); ++i)
    state_interfaces.emplace_back("axis_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &axes_[i]);
  
  for (size_t i = 0; i < buttons_.size(); ++i)
    state_interfaces.emplace_back("button_" + std::to_string(i), hardware_interface::HW_IF_POSITION, &buttons_[i]);

  return state_interfaces;
}

hardware_interface::return_type GamepadHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{ 
  (void)time;
  (void)period;
  struct js_event event;
  while (::read(fd_, &event, sizeof(event)) > 0) {
    event.type &= ~JS_EVENT_INIT;  // Ignore init events

    if (event.type == JS_EVENT_AXIS && event.number < axes_.size()) 
      axes_[event.number] = static_cast<double>(event.value) / 32767.0;

    else if (event.type == JS_EVENT_BUTTON && event.number < buttons_.size())
      buttons_[event.number] = static_cast<double>(event.value);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace gamepad