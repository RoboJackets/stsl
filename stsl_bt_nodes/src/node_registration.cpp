#include "behaviortree_cpp_v3/bt_factory.h"
#include "stsl_bt_nodes/log_node.hpp"
#include "stsl_bt_nodes/park_at_peak_node.hpp"

BT_REGISTER_NODES(factory)
{
    factory.registerBuilder<stsl_bt_nodes::LogNode>("Log", BT::CreateBuilder<stsl_bt_nodes::LogNode>());
    factory.registerBuilder<stsl_bt_nodes::ParkAtPeakNode>("ParkAtPeak", BT::CreateBuilder<stsl_bt_nodes::ParkAtPeakNode>());
}
