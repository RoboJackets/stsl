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

#include "get_sample_pose_node.hpp"
#include <stsl_interfaces/msg/mineral_deposit_sample.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace stsl_bt_nodes
{

GetSamplePose::GetSamplePose(const std::string & name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  ros_node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
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
  const auto sample = getInput<stsl_interfaces::msg::MineralDepositSample>("sample");
  if (!sample) {
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
