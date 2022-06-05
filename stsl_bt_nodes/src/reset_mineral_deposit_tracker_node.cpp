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

#include "reset_mineral_deposit_tracker_node.hpp"
#include <string>
#include <stsl_interfaces/msg/mineral_deposit_sample.hpp>

namespace stsl_bt_nodes
{

ResetMineralDepositTrackerNode::ResetMineralDepositTrackerNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: nav2_behavior_tree::BtServiceNode<stsl_interfaces::srv::ResetMineralDepositTracking>(xml_tag_name,
    conf)
{
}

BT::PortsList ResetMineralDepositTrackerNode::providedPorts()
{
  return providedBasicPorts(
    {
      BT::InputPort<stsl_interfaces::msg::MineralDepositSample>("sample")
    });
}

void ResetMineralDepositTrackerNode::on_tick()
{
  const auto sample = getInput<stsl_interfaces::msg::MineralDepositSample>("sample");
  if (!sample) {
    throw BT::RuntimeError("Missing value for required port: sample");
  }
  request_->id = sample->id;
  request_->pose = sample->pose;
}

}  // namespace stsl_bt_nodes
