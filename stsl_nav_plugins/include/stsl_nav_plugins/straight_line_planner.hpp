#include <nav2_core/global_planner.hpp>


namespace stsl_nav_plugins
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string global_frame_;
};

}