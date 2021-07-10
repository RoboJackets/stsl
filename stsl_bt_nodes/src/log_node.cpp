#include "stsl_bt_nodes/log_node.hpp"

namespace stsl_bt_nodes
{

LogNode::LogNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf)
 : BT::SyncActionNode(xml_tag_name, conf)
{
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("node");
}

BT::PortsList LogNode::providedPorts()
{
    return { BT::InputPort<std::string>("message") };
}

BT::NodeStatus LogNode::tick()
{
    BT::Optional<std::string> message = getInput<std::string>("message");
    if(!message) {
        throw BT::RuntimeError("Missing required input [message]: ", message.error());
    }
    RCLCPP_INFO(node_->get_logger(), "%s", message.value().c_str());
    return BT::NodeStatus::SUCCESS;
}


}  // namespace stsl_bt_nodes
