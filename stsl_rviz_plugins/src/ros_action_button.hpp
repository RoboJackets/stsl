// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#ifndef ROS_ACTION_BUTTON_HPP_
#define ROS_ACTION_BUTTON_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <memory>
#include <string>
#include "action_button.hpp"

namespace stsl_rviz_plugins
{

template<typename ActionType>
class RosActionButton : public ActionButton
{
public:
  RosActionButton(
    const std::string & action_name, const std::string & button_text, const std::string & node_name,
    QWidget * parent = nullptr)
  : ActionButton(button_text, parent),
    action_name_(action_name)
  {
    ros_node_ = std::make_shared<rclcpp::Node>(node_name);
    action_client_ = rclcpp_action::create_client<ActionType>(
      ros_node_,
      action_name_
    );
  }

  virtual ~RosActionButton() = default;

private:
  const std::chrono::seconds server_timeout_{1};
  const std::string action_name_;
  rclcpp::Node::SharedPtr ros_node_;
  typename rclcpp_action::Client<ActionType>::SharedPtr action_client_;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionType>;
  typename GoalHandle::SharedPtr goal_handle_;
  std::shared_future<typename GoalHandle::WrappedResult> action_result_future_;

  bool SendGoal() override
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "Action server for %s is not available.", action_name_.c_str());
      return false;
    }

    typename ActionType::Goal goal;
    auto future_goal_handle = action_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(
        ros_node_, future_goal_handle,
        server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(ros_node_->get_logger(), "Send goal call failed.");
      return false;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server.");
      return false;
    }
    action_result_future_ = action_client_->async_get_result(goal_handle_);
    return true;
  }

  void CancelGoal() override
  {
    action_client_->async_cancel_goal(goal_handle_);
  }

  ActionStatus CheckActionStatus() override
  {
    rclcpp::spin_some(ros_node_);
    if (action_result_future_.wait_for(std::chrono::milliseconds(10)) ==
      std::future_status::timeout)
    {
      return ActionStatus::Running;
    }

    const auto result = action_result_future_.get();
    switch (result.code) {
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(ros_node_->get_logger(), "Action failed.");
        return ActionStatus::Aborted;
      case rclcpp_action::ResultCode::CANCELED:
        return ActionStatus::Cancelled;
      default:
        return ActionStatus::Completed;
    }
  }
};

}  // namespace stsl_rviz_plugins

#endif  // ROS_ACTION_BUTTON_HPP_
