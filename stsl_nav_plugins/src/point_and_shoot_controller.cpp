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

#include <nav2_core/controller.hpp>
#include <stsl_nav_plugins/pd_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <angles/angles.h>
#include <memory>
#include <string>

namespace stsl_nav_plugins
{

class PointAndShootController : public nav2_core::Controller
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = node;
    auto node_shared = node_.lock();
    if(!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }
    tf_buffer_ = tf_buffer;
    const auto linear_controller_params = node_shared->declare_parameters<double>(
      name + ".linear_controller", {{"kD", 0.0}, {"kP", 1.0}});
    linear_controller_.setParameters(linear_controller_params[1], linear_controller_params[0]);
    const auto angular_controller_params = node_shared->declare_parameters<double>(
      name + ".angular_controller", {{"kD", 0.0}, {"kP", 1.0}});
    angular_controller_.setParameters(angular_controller_params[1], angular_controller_params[0]);
    yaw_threshold_ = node_shared->declare_parameter<double>(name + ".yaw_threshold", 0.04);
    xy_threshold_ = node_shared->declare_parameter<double>(name + ".xy_threshold", 0.05);
    max_x_vel_ = node_shared->declare_parameter<double>(name + ".max_x_vel", 0.3);
    max_theta_vel_ = node_shared->declare_parameter<double>(name + ".max_theta_vel", 0.1);
    match_goal_heading_ = node_shared->declare_parameter<bool>(name + ".match_goal_heading", false);
    skip_first_pose_ = node_shared->declare_parameter<bool>(name + ".skip_first_pose", true);
  }

  void activate() override {}

  void deactivate() override {}

  void cleanup() override {}

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    global_path_ = path;
    path_index_ = skip_first_pose_ ? 1 : 0;
    state_ = State::TurnToFaceGoal;
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    // TODO goal_checker added in humble upgrade

    auto node_shared = node_.lock();
    if(!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.header.stamp = node_shared->now();
    if (path_index_ >= global_path_.poses.size()) {
      return cmd_vel;
    }
    const auto goal_pose = global_path_.poses[path_index_];

    switch (state_) {
      case State::TurnToFaceGoal:
        {
          bool state_done = false;
          turnToFaceGoal(pose.pose, goal_pose.pose, cmd_vel, state_done);
          if (state_done) {
            state_ = State::MoveToGoal;
            angular_controller_.reset();
            linear_controller_.reset();
          }
          break;
        }
      case State::MoveToGoal:
        {
          bool state_done = false;
          moveToGoal(pose.pose, goal_pose.pose, cmd_vel, state_done);
          if (state_done) {
            state_ = State::TurnToMatchGoal;
            angular_controller_.reset();
            linear_controller_.reset();
          }
          break;
        }
      case State::TurnToMatchGoal:
        {
          bool state_done = false;
          turnToMatchGoal(pose.pose, goal_pose.pose, cmd_vel, state_done);
          if (state_done) {
            state_ = State::TurnToFaceGoal;
            angular_controller_.reset();
            linear_controller_.reset();
            path_index_++;
          }
          break;
        }
    }

    cmd_vel.twist.linear.x = std::clamp(cmd_vel.twist.linear.x, 0.0, max_x_vel_);
    cmd_vel.twist.angular.z = std::clamp(cmd_vel.twist.angular.z, -max_theta_vel_, max_theta_vel_);

    return cmd_vel;
  }

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override {
    // TODO implement this
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  nav_msgs::msg::Path global_path_;
  int path_index_ = 0;
  enum class State
  {
    TurnToFaceGoal,
    MoveToGoal,
    TurnToMatchGoal
  } state_ = State::TurnToFaceGoal;
  PDController linear_controller_;
  PDController angular_controller_;
  double yaw_threshold_;
  double xy_threshold_;
  double max_x_vel_;
  double max_theta_vel_;
  bool match_goal_heading_;
  bool skip_first_pose_;

  void turnToFaceGoal(
    const geometry_msgs::msg::Pose & robot_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    geometry_msgs::msg::TwistStamped & cmd_vel, bool & state_done)
  {
    const auto current_heading = tf2::getYaw(robot_pose.orientation);
    const auto target_heading = std::atan2(
      goal_pose.position.y - robot_pose.position.y,
      goal_pose.position.x - robot_pose.position.x);
    const auto heading_error = angles::shortest_angular_distance(current_heading, target_heading);
    cmd_vel.twist.angular.z = angular_controller_.step(heading_error);
    state_done = std::abs(heading_error) < yaw_threshold_;
  }

  void moveToGoal(
    const geometry_msgs::msg::Pose & robot_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    geometry_msgs::msg::TwistStamped & cmd_vel, bool & state_done)
  {
    const auto current_heading = tf2::getYaw(robot_pose.orientation);
    const auto target_heading = std::atan2(
      goal_pose.position.y - robot_pose.position.y,
      goal_pose.position.x - robot_pose.position.x);
    const auto heading_error = angles::shortest_angular_distance(current_heading, target_heading);
    cmd_vel.twist.angular.z = angular_controller_.step(heading_error);

    const auto distance = std::hypot(
      robot_pose.position.x - goal_pose.position.x,
      robot_pose.position.y - goal_pose.position.y,
      robot_pose.position.z - goal_pose.position.z);
    cmd_vel.twist.linear.x = linear_controller_.step(distance);

    state_done = distance < xy_threshold_;
  }

  void turnToMatchGoal(
    const geometry_msgs::msg::Pose & robot_pose,
    const geometry_msgs::msg::Pose & goal_pose,
    geometry_msgs::msg::TwistStamped & cmd_vel, bool & state_done)
  {
    if (!match_goal_heading_) {
      state_done = true;
      return;
    }
    const auto current_heading = tf2::getYaw(robot_pose.orientation);
    const auto target_heading = tf2::getYaw(goal_pose.orientation);
    const auto heading_error = angles::shortest_angular_distance(current_heading, target_heading);
    cmd_vel.twist.angular.z = angular_controller_.step(heading_error);
    state_done = std::abs(heading_error) < yaw_threshold_;
  }
};

}  // namespace stsl_nav_plugins

PLUGINLIB_EXPORT_CLASS(stsl_nav_plugins::PointAndShootController, nav2_core::Controller)
