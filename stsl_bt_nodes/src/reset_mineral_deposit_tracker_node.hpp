#ifndef STSL_BT_NODES__RESET_MINERAL_DEPOSIT_TRACKER_NODE_HPP_
#define STSL_BT_NODES__RESET_MINERAL_DEPOSIT_TRACKER_NODE_HPP_

#include <nav2_behavior_tree/bt_service_node.hpp>
#include <stsl_interfaces/srv/reset_mineral_deposit_tracking.hpp>

namespace stsl_bt_nodes
{

class ResetMineralDepositTrackerNode : public nav2_behavior_tree::BtServiceNode<stsl_interfaces::srv::ResetMineralDepositTracking>
{
public:
    ResetMineralDepositTrackerNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts();

    void on_tick() override;
};

}

#endif  // STSL_BT_NODES__RESET_MINERAL_DEPOSIT_TRACKER_NODE_HPP_
