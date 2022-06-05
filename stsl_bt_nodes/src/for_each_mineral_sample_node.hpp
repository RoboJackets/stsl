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

#ifndef FOR_EACH_MINERAL_SAMPLE_NODE_HPP_
#define FOR_EACH_MINERAL_SAMPLE_NODE_HPP_

#include <behaviortree_cpp_v3/decorator_node.h>
#include <string>
#include <vector>
#include <stsl_interfaces/msg/mineral_deposit_sample.hpp>

namespace stsl_bt_nodes
{

class ForEachMineralSampleNode : public BT::DecoratorNode
{
public:
  ForEachMineralSampleNode(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  void halt() override;

protected:
  BT::NodeStatus tick() override;

private:
  bool idle_{true};
  std::vector<stsl_interfaces::msg::MineralDepositSample> samples_;
  std::vector<stsl_interfaces::msg::MineralDepositSample>::iterator samples_iter_;

  void Reset();
};

}  // namespace stsl_bt_nodes

#endif  // FOR_EACH_MINERAL_SAMPLE_NODE_HPP_
