#include "remote_control/remote_control.hpp"

namespace remote_control 
{

bool RemoteControl::open_joystick()
{
  int fd = open("/dev/input/by-id/usb-Logitech_Logitech_Dual_Action_CC64C09D-event-joystick", O_RDONLY | O_NONBLOCK);
  if (fd < 0) { return false; }
  if (libevdev_new_from_fd(fd, &dev_) < 0) {
    close(fd);
    return false;
  }
  fd_ = fd;
  return true;
}

hardware_interface::CallbackReturn RemoteControl::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  axis_x_ = 0.0;
  axis_y_ = 0.0;

  if (!open_joystick()) {
    RCLCPP_ERROR(rclcpp::get_logger("RemoteControl"), "Failed to open joystick device");
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
  struct input_event ev;
  while (libevdev_next_event(dev_, LIBEVDEV_READ_FLAG_NORMAL, &ev) == 0) {
    if (ev.type == EV_ABS) {
      if (ev.code == ABS_X) {
        axis_x_ = ev.value / 32767.0;  // Normalize from -32767 to 32767
      } else 
      if (ev.code == ABS_Y) {
        axis_y_ = ev.value / 32767.0;
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("RemoteControl"), "X: %.2f Y: %.2f", axis_x_, axis_y_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type remote_control::RemoteControl::write(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  // TODO: Implement controller output if needed.
  return hardware_interface::return_type::OK;
}

RemoteControl::~RemoteControl() {
  if (dev_) libevdev_free(dev_);
  if (fd_ >= 0) close(fd_);
}

}  // namespace remote_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(remote_control::RemoteControl, hardware_interface::SystemInterface)