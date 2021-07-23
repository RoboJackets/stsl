#include "reset_mineral_deposit_tracker_node.hpp"
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

} // namespace stsl_bt_nodes
