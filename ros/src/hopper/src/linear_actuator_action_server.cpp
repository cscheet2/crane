/**
 * @see https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Creating-an-Action.html
 * @see https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
 */

#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "interfaces/action/move_linear_actuator.hpp"

#include "hopper/visibility_control.h"

namespace linear_actuator_action 
{
class LinearActuatorActionServer : public rclcpp::Node 
{
public:
  using MoveLinearActuator = interfaces::action::MoveLinearActuator;
  using GoalHandleControl = rclcpp_action::ServerGoalHandle<MoveLinearActuator>;
  CUSTOM_ACTION_CPP_PUBLIC
  
  explicit LinearActuatorActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) 
  : Node("linear_actuator_action_server", options) 
  {
    using namespace std::placeholders;

    auto handle_goal = [this](
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const MoveLinearActuator::Goal> goal) 
    {
      RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->target);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](
      const std::shared_ptr<GoalHandleControl> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](
      const std::shared_ptr<GoalHandleControl> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor,
      // so we declare a lambda function to be called inside a new thread
      auto execute_in_thread = [this, goal_handle](){return this->execute(goal_handle);};
      std::thread{execute_in_thread}.detach();
    };

    this->action_server_ = rclcpp_action::create_server<MoveLinearActuator>(
      this,
      "linear_actuator",
      handle_goal,
      handle_cancel,
      handle_accepted);
  }

private:
  rclcpp_action::Server<MoveLinearActuator>::SharedPtr action_server_;

  void execute(const std::shared_ptr<GoalHandleControl> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing r");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<MoveLinearActuator::Feedback>();
    auto result = std::make_shared<MoveLinearActuator::Result>();

    // get this to work with the linear actuator extention
    for (int8_t percent = 0; percent <= 100; percent += 20) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Canceled linear actuator movement");
        return;
      }
      // update then publish feedback
      feedback->percent_traveled = percent;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Precent of arm moved%d%%", feedback->percent_traveled);
      loop_rate.sleep();
    }
    // see if goal is met
    if (rclcpp::ok()) {
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Arm moved");
    }
  }
};
}

RCLCPP_COMPONENTS_REGISTER_NODE(linear_actuator_action::LinearActuatorActionServer)