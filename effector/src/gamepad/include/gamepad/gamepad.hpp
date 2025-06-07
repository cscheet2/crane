#ifndef _GAMEPAD_HPP
#define _GAMEPAD_HPP

#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <unistd.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gamepad
{
class GamepadHardware : public hardware_interface::SystemInterface 
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
};


}  // namespace gamepad

#endif  // _GAMEPAD_HPP