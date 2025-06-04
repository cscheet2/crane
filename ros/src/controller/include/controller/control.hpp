#ifndef _CONTROL_H
#define _CONTROL_H

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/drive_train.hpp"
#include "interfaces/msg/excavation.hpp"
#include "interfaces/msg/hopper.hpp"
#include "interfaces/msg/vision.hpp"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node 
{
public:
  Controller();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<interfaces::msg::DriveTrain>::SharedPtr drivetrain_publisher_;
  size_t count_;
};

#endif  // _CONTROL_H