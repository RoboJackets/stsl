#include "get_sample_pose_node.hpp"
#include <stsl_interfaces/msg/mineral_deposit_sample.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace stsl_bt_nodes
{

GetSamplePose::GetSamplePose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::PortsList GetSamplePose::providedPorts()
{
  return {
    BT::InputPort<stsl_interfaces::msg::MineralDepositSample>("sample"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose")
  };
}

BT::NodeStatus GetSamplePose::tick()
{
  if(!ros_node_) {
    ros_node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  }
  const auto sample = getInput<stsl_interfaces::msg::MineralDepositSample>("sample");
  if(!sample) {
    throw BT::RuntimeError("Missing required port: sample");
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = ros_node_->now();
  pose.header.frame_id = "map";
  pose.pose = sample->pose.pose;
  setOutput("pose", pose);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace stsl_bt_nodes
