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

#include <memory>
#include <string>
#include <nav2_core/global_planner.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace stsl_nav_plugins
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = node;
    global_frame_ = costmap_ros->getGlobalFrameID();
  }

  void cleanup() override {}

  void activate() override {}

  void deactivate() override {}

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override
  {
    nav_msgs::msg::Path path;

    auto node_shared = node_.lock();
    if (!node_shared) {
      throw std::runtime_error{"Could not acquire node."};
    }

    if (start.header.frame_id != global_frame_) {
      RCLCPP_ERROR(
        node_shared->get_logger(), "Planner will only except start position from %s frame",
        global_frame_.c_str());
      return path;
    }

    if (goal.header.frame_id != global_frame_) {
      RCLCPP_INFO(
        node_shared->get_logger(), "Planner will only except goal position from %s frame",
        global_frame_.c_str());
      return path;
    }

    path.header.stamp = node_shared->now();
    path.header.frame_id = global_frame_;
    path.poses = {start, goal};

    return path;
  }

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string global_frame_;
};

}  // namespace stsl_nav_plugins

PLUGINLIB_EXPORT_CLASS(stsl_nav_plugins::StraightLinePlanner, nav2_core::GlobalPlanner)
