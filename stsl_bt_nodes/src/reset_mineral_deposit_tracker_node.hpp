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

#ifndef RESET_MINERAL_DEPOSIT_TRACKER_NODE_HPP_
#define RESET_MINERAL_DEPOSIT_TRACKER_NODE_HPP_

#include <string>
#include <nav2_behavior_tree/bt_service_node.hpp>
#include <stsl_interfaces/srv/reset_mineral_deposit_tracking.hpp>

namespace stsl_bt_nodes
{

class ResetMineralDepositTrackerNode
  : public nav2_behavior_tree::BtServiceNode<stsl_interfaces::srv::ResetMineralDepositTracking>
{
public:
  ResetMineralDepositTrackerNode(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  void on_tick() override;
};

}  // namespace stsl_bt_nodes

#endif  // RESET_MINERAL_DEPOSIT_TRACKER_NODE_HPP_
