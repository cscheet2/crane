#ifndef _REMOTE_CONTROL_HPP
#define _REMOTE_CONTROL_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"

namespace remote_control
{

class RemoteControl : public hardware_interface::SystemInterface
{
public:  
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  double axis_x_ = 0.0;
  double axis_y_ = 0.0;
};

}  // namespace remote_control

#endif  // _REMOTE_CONTROL_HPP