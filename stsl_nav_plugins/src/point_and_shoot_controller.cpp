#include "stsl_nav_plugins/point_and_shoot_controller.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace stsl_nav_plugins
{
void PointAndShootController::configure(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, std::string name,
    const std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>& costmap)
{
  node_ = node;
  tf_buffer_ = tf_buffer;
  const auto linear_controller_params = node_->declare_parameters<double>(
      name + ".linear_controller", { { "kD", 0.0 }, { "kP", 1.0 } });
  linear_controller_.setParameters(linear_controller_params[1], linear_controller_params[0]);
  const auto angular_controller_params = node_->declare_parameters<double>(
      name + ".angular_controller", { { "kD", 0.0 }, { "kP", 1.0 } });
  angular_controller_.setParameters(angular_controller_params[1], angular_controller_params[0]);
  yaw_threshold_ = node_->declare_parameter<double>("yaw_threshold", 0.04);
  xy_threshold_ = node_->declare_parameter<double>("xy_threshold", 0.05);
  max_x_vel_ = node_->declare_parameter<double>("max_x_vel", 0.3);
  max_theta_vel_ = node_->declare_parameter<double>("max_theta_vel", 0.1);
  match_goal_heading_ = node_->declare_parameter<double>("match_goal_heading", false);
}

void PointAndShootController::activate()
{
}

void PointAndShootController::deactivate()
{
}

void PointAndShootController::cleanup()
{
}

void PointAndShootController::setPlan(const nav_msgs::msg::Path& path)
{
  global_path_ = path;
  path_index_ = 0;
}

geometry_msgs::msg::TwistStamped PointAndShootController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = "base_link";
  cmd_vel.header.stamp = node_->now();
  if (path_index_ >= global_path_.poses.size())
  {
    return cmd_vel;
  }
  const auto goal_pose = global_path_.poses[path_index_];

  switch (state_)
  {
    case State::TurnToFaceGoal:
    {
      RCLCPP_INFO(node_->get_logger(), "State = TurnToFaceGoal");
      bool state_done = false;
      turnToFaceGoal(pose.pose, goal_pose.pose, cmd_vel, state_done);
      if(state_done)
      {
        state_ = State::MoveToGoal;
        angular_controller_.reset();
        linear_controller_.reset();
      }
      break;
    }
    case State::MoveToGoal:
    {
      RCLCPP_INFO(node_->get_logger(), "State = MoveToGoal");
      bool state_done = false;
      moveToGoal(pose.pose, goal_pose.pose, cmd_vel, state_done);
      if(state_done)
      {
        state_ = State::TurnToMatchGoal;
        angular_controller_.reset();
        linear_controller_.reset();
      }
      break;
    }
    case State::TurnToMatchGoal:
    {
      RCLCPP_INFO(node_->get_logger(), "State = TurnToMatchGoal");
      bool state_done = false;
      turnToMatchGoal(pose.pose, goal_pose.pose, cmd_vel, state_done);
      if(state_done) {
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

void PointAndShootController::turnToFaceGoal(const geometry_msgs::msg::Pose& robot_pose,
                                             const geometry_msgs::msg::Pose& goal_pose,
                                             geometry_msgs::msg::TwistStamped& cmd_vel,
                                             bool& state_done)
{
  const auto current_heading = tf2::getYaw(robot_pose.orientation);
  const auto target_heading = std::atan2(goal_pose.position.y - robot_pose.position.y, goal_pose.position.x - robot_pose.position.x);
  const auto heading_error = target_heading - current_heading;
  RCLCPP_INFO(node_->get_logger(), "Heading error: %f", heading_error);
  cmd_vel.twist.angular.z = angular_controller_.step(heading_error);
  RCLCPP_INFO(node_->get_logger(), "Z vel: %f", cmd_vel.twist.angular.z);
  state_done = std::abs(heading_error) < yaw_threshold_;
}

void PointAndShootController::moveToGoal(const geometry_msgs::msg::Pose& robot_pose,
                                         const geometry_msgs::msg::Pose& goal_pose,
                                         geometry_msgs::msg::TwistStamped& cmd_vel,
                                         bool& state_done)
{
  const auto current_heading = tf2::getYaw(robot_pose.orientation);
  const auto target_heading = std::atan2(goal_pose.position.y - robot_pose.position.y, goal_pose.position.x - robot_pose.position.x);
  const auto heading_error = target_heading - current_heading;
  cmd_vel.twist.angular.z = angular_controller_.step(heading_error);
  
  const auto distance = std::hypot(robot_pose.position.x - goal_pose.position.x, robot_pose.position.y - goal_pose.position.y, robot_pose.position.z - goal_pose.position.z);
  cmd_vel.twist.linear.x = linear_controller_.step(distance);
  RCLCPP_INFO(node_->get_logger(), "Distance: %f", distance);
  RCLCPP_INFO(node_->get_logger(), "X vel: %f", cmd_vel.twist.linear.x);
  
  state_done = distance < xy_threshold_;
}

void PointAndShootController::turnToMatchGoal(const geometry_msgs::msg::Pose& robot_pose,
                                              const geometry_msgs::msg::Pose& goal_pose,
                                              geometry_msgs::msg::TwistStamped& cmd_vel,
                                              bool& state_done)
{
  if(!match_goal_heading_)
  {
    state_done = true;
    return;
  }
  const auto current_heading = tf2::getYaw(robot_pose.orientation);
  const auto target_heading = tf2::getYaw(goal_pose.orientation);
  const auto heading_error = target_heading - current_heading;
  cmd_vel.twist.angular.z = angular_controller_.step(heading_error);
  state_done = std::abs(heading_error) < yaw_threshold_;
}

}  // namespace stsl_nav_plugins

PLUGINLIB_EXPORT_CLASS(stsl_nav_plugins::PointAndShootController, nav2_core::Controller)
