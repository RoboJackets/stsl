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

#include "park_at_peak_button.hpp"
#include <QVBoxLayout>
#include <QFont>
#include <pluginlib/class_list_macros.hpp>
#include <memory>

namespace stsl_rviz_plugins
{

ParkAtPeakButton::ParkAtPeakButton(QWidget * parent)
: rviz_common::Panel(parent),
  timer_(new QTimer(this)),
  button_(new QPushButton),
  idle_state_(new QState),
  running_state_(new QState),
  cancelling_state_(new QState)
{
  button_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  QFont button_font = button_->font();
  button_font.setPointSize(20);
  button_->setFont(button_font);

  idle_state_->setObjectName("idle");
  idle_state_->assignProperty(button_, "text", "Park at Peak");
  idle_state_->assignProperty(button_, "enabled", true);
  idle_state_->addTransition(button_, SIGNAL(clicked()), running_state_);

  running_state_->setObjectName("running");
  running_state_->assignProperty(button_, "text", "Cancel");
  running_state_->assignProperty(button_, "enabled", true);
  running_state_->addTransition(button_, SIGNAL(clicked()), cancelling_state_);
  running_state_->addTransition(this, SIGNAL(runningCompleted()), idle_state_);
  QObject::connect(running_state_, SIGNAL(entered()), this, SLOT(onEnterRunning()));

  cancelling_state_->setObjectName("cancelling");
  cancelling_state_->assignProperty(button_, "text", "Cancelling...");
  cancelling_state_->assignProperty(button_, "enabled", false);
  cancelling_state_->addTransition(this, SIGNAL(cancellingCompleted()), idle_state_);
  QObject::connect(cancelling_state_, SIGNAL(entered()), this, SLOT(onEnterCancelling()));

  state_machine_.addState(idle_state_);
  state_machine_.addState(running_state_);
  state_machine_.addState(cancelling_state_);
  state_machine_.setInitialState(idle_state_);
  state_machine_.start();

  QVBoxLayout * layout = new QVBoxLayout;
  layout->addWidget(button_);
  setLayout(layout);

  ros_node_ = std::make_shared<rclcpp::Node>("rviz_park_at_peak_client");

  action_client_ = rclcpp_action::create_client<stsl_interfaces::action::ParkAtPeak>(
    ros_node_,
    "park_at_peak"
  );
}

void ParkAtPeakButton::onEnterRunning()
{
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(ros_node_->get_logger(), "ParkAtPeak action server is not available.");
    runningCompleted();
    return;
  }

  stsl_interfaces::action::ParkAtPeak::Goal goal;
  auto future_goal_handle = action_client_->async_send_goal(goal);
  if (rclcpp::spin_until_future_complete(
      ros_node_, future_goal_handle,
      server_timeout_) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(ros_node_->get_logger(), "Send goal call failed.");
    runningCompleted();
    return;
  }

  goal_handle_ = future_goal_handle.get();
  if (!goal_handle_) {
    RCLCPP_ERROR(ros_node_->get_logger(), "Goal was rejected by server.");
    runningCompleted();
    return;
  }
  action_result_future_ = action_client_->async_get_result(goal_handle_);

  timer_connection_ = timer_->callOnTimeout(this, &ParkAtPeakButton::CheckActionStatus);
  timer_->start(100);
}

void ParkAtPeakButton::onEnterCancelling()
{
  action_client_->async_cancel_goal(goal_handle_);
}

void ParkAtPeakButton::CheckActionStatus()
{
  rclcpp::spin_some(ros_node_);

  if (action_result_future_.wait_for(std::chrono::milliseconds(10)) ==
    std::future_status::timeout)
  {
    return;
  }

  const auto result = action_result_future_.get();
  switch (result.code) {
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(ros_node_->get_logger(), "ParkAtPeak action failed.");
      runningCompleted();
      break;
    case rclcpp_action::ResultCode::CANCELED:
      cancellingCompleted();
      break;
    default:
      runningCompleted();
      break;
  }
  timer_->stop();
}

}  // namespace stsl_rviz_plugins

PLUGINLIB_EXPORT_CLASS(stsl_rviz_plugins::ParkAtPeakButton, rviz_common::Panel)
