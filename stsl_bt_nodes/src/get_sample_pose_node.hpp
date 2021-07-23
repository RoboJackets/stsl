#ifndef GET_SAMPLE_POSE_NODE_HPP_
#define GET_SAMPLE_POSE_NODE_HPP_
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace stsl_bt_nodes
{

class GetSamplePose : public BT::SyncActionNode
{
public:
  GetSamplePose(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr ros_node_;
};

}  // namespace stsl_bt_nodes

#endif  // GET_SAMPLE_POSE_NODE_HPP_
