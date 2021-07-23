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

#include "log_node.hpp"
#include <string>

namespace stsl_bt_nodes
{

LogNode::LogNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
: BT::SyncActionNode(xml_tag_name, conf)
{
  node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
  std::string topic;
  node_->get_parameter("goal_updater_topic", topic);
  RCLCPP_ERROR(node_->get_logger(), "Topic param value: %s", topic.c_str());
}

BT::PortsList LogNode::providedPorts()
{
  return {BT::InputPort<std::string>("message")};
}

BT::NodeStatus LogNode::tick()
{
  BT::Optional<std::string> message = getInput<std::string>("message");
  if (!message) {
    throw BT::RuntimeError("Missing required input [message]: ", message.error());
  }
  RCLCPP_INFO(node_->get_logger(), "%s", message.value().c_str());
  return BT::NodeStatus::SUCCESS;
}


}  // namespace stsl_bt_nodes
