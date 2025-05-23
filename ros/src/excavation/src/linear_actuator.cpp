#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/hopper.hpp"

/**
 * Enum to keep track of the current hopper state.
 */
enum LINEAR_ACTUATOR_STATE : int8_t {
  RETRACTING = -1,
  RESTING    =  0,
  EXTENDING  =  1,
};

class LinearActuator : public rclcpp::Node {
  public:
    LinearActuator() : Node("linear_actuator") {
      
    }
  private:
    enum LINEAR_ACTUATOR_STATE state = RESTING;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearActuator>());
  rclcpp::shutdown();
  return 0;
}