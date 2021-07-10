#ifndef STSL_BT_NODES__PARK_AT_PEAK_NODE_HPP_
#define STSL_BT_NODES__PARK_AT_PEAK_NODE_HPP_

#include <stsl_interfaces/action/park_at_peak.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>

namespace stsl_bt_nodes
{

class ParkAtPeakNode : public nav2_behavior_tree::BtActionNode<stsl_interfaces::action::ParkAtPeak>
{
public:
  ParkAtPeakNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
  : nav2_behavior_tree::BtActionNode<stsl_interfaces::action::ParkAtPeak>(xml_tag_name,
      "/park_at_peak", conf)
  {
  }
};

}  // namespace stsl_bt_nodes

#endif  // STSL_BT_NODES__PARK_AT_PEAK_NODE_HPP_
