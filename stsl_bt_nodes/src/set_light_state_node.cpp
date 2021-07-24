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

#include "set_light_state_node.hpp"
#include <string>

namespace stsl_bt_nodes
{

SetLightStateNode::SetLightStateNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  ros_node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  publisher_ = ros_node_->create_publisher<std_msgs::msg::Bool>(
    "/light_control/indicator_light",
    rclcpp::SystemDefaultsQoS());
}

BT::PortsList SetLightStateNode::providedPorts()
{
  return {
    BT::InputPort<bool>("state")
  };
}

BT::NodeStatus SetLightStateNode::tick()
{
  const auto state = getInput<bool>("state");
  if (!state.has_value()) {
    throw BT::RuntimeError("Missing required port: state");
  }
  std_msgs::msg::Bool msg;
  msg.data = state.value();
  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}
}  // namespace stsl_bt_nodes
