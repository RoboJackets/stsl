#include <nav2_core/progress_checker.hpp>

namespace stsl_nav_plugins
{
class NoProgressChecker : public nav2_core::ProgressChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                  const std::string& plugin_name) override;
  bool check(geometry_msgs::msg::PoseStamped& current_pose) override;
  void reset() override;

private:
};

}  // namespace stsl_nav_plugins