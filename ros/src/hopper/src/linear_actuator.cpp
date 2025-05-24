#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/msg/hopper.hpp"
#include "interfaces/action/move_linear_actuator.hpp"

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
    // using Control = interfaces::action::MoveLinearActuator;
    // using GoalHandleControl = rclcpp_action::ServerGoalHandle<MoveLinearActuator>;
    LinearActuator() : Node("linear_actuator"), state_(RESTING) {
      using namespace std::placeholders;
      
    }
  private:
    
    LINEAR_ACTUATOR_STATE state_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LinearActuator>());
  rclcpp::shutdown();
  return 0;
}