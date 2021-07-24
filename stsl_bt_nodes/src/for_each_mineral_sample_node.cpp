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

#include "for_each_mineral_sample_node.hpp"
#include <string>
#include <vector>

namespace stsl_bt_nodes
{

ForEachMineralSampleNode::ForEachMineralSampleNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
}

BT::PortsList ForEachMineralSampleNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<stsl_interfaces::msg::MineralDepositSample>>("samples_list"),
    BT::OutputPort<stsl_interfaces::msg::MineralDepositSample>("current_sample")
  };
}

void ForEachMineralSampleNode::halt()
{
  Reset();
  BT::DecoratorNode::halt();
}

BT::NodeStatus ForEachMineralSampleNode::tick()
{
  if (idle_) {
    BT::Optional<std::vector<stsl_interfaces::msg::MineralDepositSample>> samples_port =
      getInput<std::vector<stsl_interfaces::msg::MineralDepositSample>>("samples_list");
    if (!samples_port) {
      throw BT::RuntimeError("Missing required input [samples_list]: ", samples_port.error());
    }
    samples_ = samples_port.value();
    samples_iter_ = samples_.begin();
    idle_ = false;
    if (samples_.empty()) {
      Reset();
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("current_sample", *samples_iter_);
  }

  const auto child_status = child_node_->executeTick();

  switch (child_status) {
    case BT::NodeStatus::SUCCESS:
      ++samples_iter_;
      if (samples_iter_ == samples_.end()) {
        Reset();
        return BT::NodeStatus::SUCCESS;
      }
      setOutput("current_sample", *samples_iter_);
      return BT::NodeStatus::RUNNING;
    case BT::NodeStatus::RUNNING:
      return BT::NodeStatus::RUNNING;
    default:
      Reset();
      return BT::NodeStatus::FAILURE;
  }
}

void ForEachMineralSampleNode::Reset()
{
  idle_ = true;
  samples_.clear();
  samples_iter_ = std::vector<stsl_interfaces::msg::MineralDepositSample>::iterator{};
}

}  // namespace stsl_bt_nodes
