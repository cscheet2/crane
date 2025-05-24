#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "interfaces/action/move_linear_actuator.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace linear_actuator_action
{
class LinearActuatorActionClient : public rclcpp::Node
{

public:
  using MoveLinearActuator = interfaces::action::MoveLinearActuator;
  using GoalHandleControl = rclcpp_action::ClientGoalHandle<MoveLinearActuator>;
  
  explicit LinearActuatorActionClient(const rclcpp::NodeOptions & options)
  : Node("linear_actuator_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<MoveLinearActuator>(
      this,
      "move_linear_actuator");
    auto timer_callback_lambda = [this](){ return this->send_goal(); };
    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      timer_callback_lambda);
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();
    
    if(!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = MoveLinearActuator::Goal();
    // define here what position it should be moved to
    goal_msg.target = 1;

    RCLCPP_INFO(this->get_logger(), "Sending actuator change position");

    auto send_goal_options = rclcpp_action::Client<MoveLinearActuator>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleControl::SharedPtr & goal_handle)
    {
      if(!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    };

    send_goal_options.feedback_callback = [this](
      GoalHandleControl::SharedPtr,
      const std::shared_ptr<const MoveLinearActuator::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "percent finished %d", feedback->percent_traveled);
    };

    send_goal_options.result_callback = [this](const GoalHandleControl::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
          return;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
          return;
        default:
          RCLCPP_ERROR(this->get_logger(), "Unknown result code");
          return;
      }
      RCLCPP_INFO(this->get_logger(), "Result received: %d", result.result->success);
      rclcpp::shutdown();
    };
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<MoveLinearActuator>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}

RCLCPP_COMPONENTS_REGISTER_NODE(linear_actuator_action::LinearActuatorActionClient)