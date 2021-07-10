#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <tf2_ros/transform_listener.h>
#include <stsl_interfaces/action/execute_mission.hpp>
#include <fstream>

namespace mission_orchestration
{

class MissionOrchestrator : public rclcpp::Node
{
public:
  explicit MissionOrchestrator(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mission_orchestrator", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(*tf_buffer_)
  {
    action_server_ = rclcpp_action::create_server<stsl_interfaces::action::ExecuteMission>(
      this,
      "execute_mission",
      std::bind(&MissionOrchestrator::OnGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MissionOrchestrator::OnCancel, this, std::placeholders::_1),
      std::bind(&MissionOrchestrator::OnAccepted, this, std::placeholders::_1));

    const std::vector<std::string> default_plugin_libs = {
      "nav2_compute_path_to_pose_action_bt_node",
      "nav2_follow_path_action_bt_node",
      "nav2_back_up_action_bt_node",
      "nav2_spin_action_bt_node",
      "nav2_wait_action_bt_node",
      "nav2_clear_costmap_service_bt_node",
      "nav2_is_stuck_condition_bt_node",
      "nav2_goal_reached_condition_bt_node",
      "nav2_goal_updated_condition_bt_node",
      "nav2_initial_pose_received_condition_bt_node",
      "nav2_reinitialize_global_localization_service_bt_node",
      "nav2_rate_controller_bt_node",
      "nav2_distance_controller_bt_node",
      "nav2_speed_controller_bt_node",
      "nav2_truncate_path_action_bt_node",
      "nav2_goal_updater_node_bt_node",
      "nav2_recovery_node_bt_node",
      "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node",
      "nav2_transform_available_condition_bt_node",
      "nav2_time_expired_condition_bt_node",
      "nav2_distance_traveled_condition_bt_node",
      "stsl_bt_nodes"
    };

    const auto plugin_libs = declare_parameter("plugin_libs", default_plugin_libs);
    bt_engine_ = std::make_unique<nav2_behavior_tree::BehaviorTreeEngine>(plugin_libs);

    tree_ros_node_ = std::make_shared<rclcpp::Node>("mission_orchestrator_behavior_tree", options);

    blackboard_ = BT::Blackboard::create();

    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);
    blackboard_->set<rclcpp::Node::SharedPtr>("node", tree_ros_node_);
    blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));

    const auto bt_file_path = declare_parameter("bt_file_path", "NO_BT_FILE_PATH_SET");

    if (!LoadBehaviorTree(bt_file_path)) {
      RCLCPP_ERROR(get_logger(), "Error loading behavior tree from file: %s", bt_file_path.c_str());
      exit(1);
    }
  }

  bool LoadBehaviorTree(const std::string & bt_file_path)
  {
    std::ifstream bt_file{bt_file_path};

    if (!bt_file.good()) {
      RCLCPP_ERROR(get_logger(), "Could not open file: %s", bt_file_path.c_str());
      return false;
    }

    const auto xml_content = std::string(
      std::istreambuf_iterator<char>(
        bt_file), std::istreambuf_iterator<char>());

    try {
    tree_ = bt_engine_->createTreeFromText(xml_content, blackboard_);
    }catch (const BT::RuntimeError& e) {
        RCLCPP_ERROR(get_logger(), "BT exception: %s", e.what());
        return false;
    }

    return true;
  }

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_engine_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<stsl_interfaces::action::ExecuteMission>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr tree_ros_node_;

  rclcpp_action::GoalResponse OnGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const stsl_interfaces::action::ExecuteMission::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse OnCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<stsl_interfaces::action::ExecuteMission>> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void OnAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<stsl_interfaces::action::ExecuteMission>> goal_handle)
  {
    std::thread{std::bind(&MissionOrchestrator::Execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void Execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<stsl_interfaces::action::ExecuteMission>> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing mission!");

    auto is_cancelling = [goal_handle]() {
        return goal_handle->is_canceling();
      };

    auto on_loop = []() {};

    const auto tree_status = bt_engine_->run(&tree_, on_loop, is_cancelling);

    bt_engine_->haltAllActions(tree_.rootNode());

    switch (tree_status) {
      case nav2_behavior_tree::BtStatus::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Mission succeeded!");
        goal_handle->succeed(std::make_shared<stsl_interfaces::action::ExecuteMission::Result>());
        break;
      case nav2_behavior_tree::BtStatus::FAILED:
        RCLCPP_INFO(get_logger(), "Mission failed!");
        goal_handle->abort(std::make_shared<stsl_interfaces::action::ExecuteMission::Result>());
        break;
      case nav2_behavior_tree::BtStatus::CANCELED:
        RCLCPP_INFO(get_logger(), "Mission cancelled!");
        goal_handle->canceled(std::make_shared<stsl_interfaces::action::ExecuteMission::Result>());
        break;
    }
  }

};

}

RCLCPP_COMPONENTS_REGISTER_NODE(mission_orchestration::MissionOrchestrator)
