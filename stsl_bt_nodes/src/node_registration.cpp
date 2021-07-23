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

#include "behaviortree_cpp_v3/bt_factory.h"
#include "for_each_mineral_sample_node.hpp"
#include "log_node.hpp"
#include "park_at_peak_node.hpp"
#include "reset_mineral_deposit_tracker_node.hpp"
#include "get_sample_pose_node.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerBuilder<stsl_bt_nodes::ForEachMineralSampleNode>(
    "ForEachMineralSample",
    BT::CreateBuilder<stsl_bt_nodes::ForEachMineralSampleNode>());
  factory.registerBuilder<stsl_bt_nodes::LogNode>(
    "Log",
    BT::CreateBuilder<stsl_bt_nodes::LogNode>());
  factory.registerBuilder<stsl_bt_nodes::ParkAtPeakNode>(
    "ParkAtPeak",
    BT::CreateBuilder<stsl_bt_nodes::ParkAtPeakNode>());
  factory.registerBuilder<stsl_bt_nodes::ResetMineralDepositTrackerNode>(
    "ResetMineralDepositTracker",
    BT::CreateBuilder<stsl_bt_nodes::ResetMineralDepositTrackerNode>());
  factory.registerBuilder<stsl_bt_nodes::GetSamplePose>(
    "GetSamplePose",
    BT::CreateBuilder<stsl_bt_nodes::GetSamplePose>());
}
