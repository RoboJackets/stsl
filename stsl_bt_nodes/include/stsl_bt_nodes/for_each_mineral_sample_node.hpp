#ifndef STSL_BT_NODES__FOR_EACH_MINERAL_SAMPLE_NODE_HPP_
#define STSL_BT_NODES__FOR_EACH_MINERAL_SAMPLE_NODE_HPP_

#include <behaviortree_cpp_v3/decorator_node.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

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
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> samples_;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>::iterator samples_iter_;

    void Reset();
};

}  // namespace stsl_bt_nodes

#endif  // STSL_BT_NODES__FOR_EACH_MINERAL_SAMPLE_NODE_HPP_
