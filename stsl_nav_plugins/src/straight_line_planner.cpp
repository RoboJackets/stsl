#include "stsl_nav_plugins/straight_line_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace stsl_nav_plugins
{
void StraightLinePlanner::configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
                                    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = node;
  global_frame_ = costmap_ros->getGlobalFrameID();
}

void StraightLinePlanner::cleanup()
{
}

void StraightLinePlanner::activate()
{
}

void StraightLinePlanner::deactivate()
{
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(const geometry_msgs::msg::PoseStamped& start,
                                                    const geometry_msgs::msg::PoseStamped& goal)
{
  nav_msgs::msg::Path path;

  if (start.header.frame_id != global_frame_)
  {
    RCLCPP_ERROR(node_->get_logger(), "Planner will only except start position from %s frame",
                 global_frame_.c_str());
    return path;
  }

  if (goal.header.frame_id != global_frame_)
  {
    RCLCPP_INFO(node_->get_logger(), "Planner will only except goal position from %s frame",
                global_frame_.c_str());
    return path;
  }

  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;
  path.poses = {start, goal};

  return path;
}

}  // namespace stsl_nav_plugins

PLUGINLIB_EXPORT_CLASS(stsl_nav_plugins::StraightLinePlanner, nav2_core::GlobalPlanner)
