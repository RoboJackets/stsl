#ifndef STSL_NAV_PLUGINS_POINT_AND_SHOOT_CONTROLLER_HPP
#define STSL_NAV_PLUGINS_POINT_AND_SHOOT_CONTROLLER_HPP

#include <nav2_core/controller.hpp>
#include <stsl_nav_plugins/pd_controller.hpp>

namespace stsl_nav_plugins
{
class PointAndShootController : public nav2_core::Controller
{
public:
    void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap) override;

    void activate() override;

    void deactivate() override;

    void cleanup() override;

    void setPlan(const nav_msgs::msg::Path& path) override;

    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::Twist& velocity) override;

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    nav_msgs::msg::Path global_path_;
    int path_index_ = 0;
    enum class State {
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

    void turnToFaceGoal(const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::Pose& goal_pose, geometry_msgs::msg::TwistStamped& cmd_vel, bool& state_done);
    void moveToGoal(const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::Pose& goal_pose, geometry_msgs::msg::TwistStamped& cmd_vel, bool& state_done);
    void turnToMatchGoal(const geometry_msgs::msg::Pose& robot_pose, const geometry_msgs::msg::Pose& goal_pose, geometry_msgs::msg::TwistStamped& cmd_vel, bool& state_done);
};

}  // namespace stsl_nav_plugins

#endif
