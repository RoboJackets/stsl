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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <nav2_behavior_tree/behavior_tree_engine.hpp>
#include <stsl_interfaces/action/execute_mission.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include "yaml_helpers.hpp"

namespace mission_orchestration
{

class MissionOrchestrator : public rclcpp::Node
{
public:
  using ExecuteMission = stsl_interfaces::action::ExecuteMission;
  using ServerGoalHandle = rclcpp_action::ServerGoalHandle<ExecuteMission>;

  explicit MissionOrchestrator(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mission_orchestrator", options),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock())),
    tf_listener_(*tf_buffer_)
  {
    action_server_ = rclcpp_action::create_server<ExecuteMission>(
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

    rclcpp::NodeOptions tree_node_options = options;
    tree_node_options.allow_undeclared_parameters(true);
    tree_node_options.automatically_declare_parameters_from_overrides(true);
    tree_ros_node_ = std::make_shared<rclcpp::Node>(
      "mission_orchestrator_behavior_tree",
      tree_node_options);

    const auto mineral_samples_file = declare_parameter<std::string>("mineral_samples_file", "");

    std::vector<stsl_interfaces::msg::MineralDepositSample> mineral_samples;
    if (!mineral_samples_file.empty() &&
      !LoadMineralSamplesFile(mineral_samples_file, mineral_samples))
    {
      RCLCPP_ERROR(get_logger(), "Failed to load mineral samples file.");
      exit(1);
    }

    blackboard_ = BT::Blackboard::create();

    blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);
    blackboard_->set<rclcpp::Node::SharedPtr>("node", tree_ros_node_);
    blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(10));
    blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10));
    blackboard_->set<std::vector<stsl_interfaces::msg::MineralDepositSample>>(
      "mineral_samples",
      mineral_samples);

    const auto bt_file_path = declare_parameter("bt_file_path", "NO_BT_FILE_PATH_SET");

    if (!LoadBehaviorTree(bt_file_path)) {
      RCLCPP_ERROR(get_logger(), "Error loading behavior tree from file: %s", bt_file_path.c_str());
      exit(1);
    }
  }

private:
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_engine_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp_action::Server<ExecuteMission>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr tree_ros_node_;

  rclcpp_action::GoalResponse OnGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const ExecuteMission::Goal> goal)
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse OnCancel(
    const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void OnAccepted(
    const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    std::thread{std::bind(&MissionOrchestrator::Execute, this, std::placeholders::_1),
      goal_handle}.detach();
  }

  void Execute(
    const std::shared_ptr<ServerGoalHandle> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing mission!");

    auto is_cancelling = [goal_handle]() {
        return goal_handle->is_canceling();
      };

    auto on_loop = []() {};

    try {
      const auto tree_status = bt_engine_->run(&tree_, on_loop, is_cancelling);

      bt_engine_->haltAllActions(tree_.rootNode());

      switch (tree_status) {
        case nav2_behavior_tree::BtStatus::SUCCEEDED:
          RCLCPP_INFO(get_logger(), "Mission succeeded!");
          goal_handle->succeed(std::make_shared<ExecuteMission::Result>());
          break;
        case nav2_behavior_tree::BtStatus::FAILED:
          RCLCPP_INFO(get_logger(), "Mission failed!");
          goal_handle->abort(std::make_shared<ExecuteMission::Result>());
          break;
        case nav2_behavior_tree::BtStatus::CANCELED:
          RCLCPP_INFO(get_logger(), "Mission cancelled!");
          goal_handle->canceled(std::make_shared<ExecuteMission::Result>());
          break;
      }
    } catch (const BT::RuntimeError & e) {
      RCLCPP_ERROR(get_logger(), "Mission failed with exception: %s", e.what());
      goal_handle->abort(std::make_shared<ExecuteMission::Result>());
      return;
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
    } catch (const BT::RuntimeError & e) {
      RCLCPP_ERROR(get_logger(), "Could not create behavior tree. Details: %s", e.what());
      return false;
    }

    return true;
  }

  bool LoadMineralSamplesFile(
    const std::string & mineral_samples_file_path,
    std::vector<stsl_interfaces::msg::MineralDepositSample> & samples)
  {
    try {
      YAML::Node root_yaml_node = YAML::LoadFile(mineral_samples_file_path);

      if (!root_yaml_node.IsSequence()) {
        RCLCPP_ERROR(get_logger(), "Root of mineral samples file should be a sequence.");
        return false;
      }

      std::transform(
        root_yaml_node.begin(), root_yaml_node.end(), std::back_inserter(
          samples), yaml_helpers::fromYaml<stsl_interfaces::msg::MineralDepositSample>);

      RCLCPP_INFO(get_logger(), "Loaded %ld mineral samples.", samples.size());

      return true;
    } catch (const YAML::BadFile & e) {
      RCLCPP_ERROR(
        get_logger(), "Could not open mineral samples file (%s): %s",
        mineral_samples_file_path.c_str(), e.what());
      return false;
    } catch (const YAML::ParserException & e) {
      RCLCPP_ERROR(
        get_logger(), "Could not parse mineral samples file (%s): %s",
        mineral_samples_file_path.c_str(), e.what());
      return false;
    }
  }
};

}  // namespace mission_orchestration

RCLCPP_COMPONENTS_REGISTER_NODE(mission_orchestration::MissionOrchestrator)
