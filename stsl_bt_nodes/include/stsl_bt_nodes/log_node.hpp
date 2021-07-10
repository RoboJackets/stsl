#ifndef STSL_BT_NODES__LOG_NODE_HPP_
#define STSL_BT_NODES__LOG_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/action_node.h>

namespace stsl_bt_nodes
{

class LogNode : public BT::SyncActionNode
{
public:
    LogNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts();

protected:
    BT::NodeStatus tick() override;

private:
    rclcpp::Node::SharedPtr node_;

};

}  // namespace stsl_bt_nodes

#endif  // STSL_BT_NODES__LOG_NODE_HPP_