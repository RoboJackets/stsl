#include "stsl_nav_plugins/no_progress_checker.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace stsl_nav_plugins
{
void NoProgressChecker::initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                   const std::string& plugin_name)
{
}
bool NoProgressChecker::check(geometry_msgs::msg::PoseStamped& current_pose)
{
    return true;
}
void NoProgressChecker::reset()
{
}
}  // namespace stsl_nav_plugins

PLUGINLIB_EXPORT_CLASS(stsl_nav_plugins::NoProgressChecker, nav2_core::ProgressChecker)
