#include "stsl_bt_nodes/for_each_mineral_sample_node.hpp"

namespace stsl_bt_nodes
{

ForEachMineralSampleNode::ForEachMineralSampleNode(const std::string & name, const BT::NodeConfiguration & conf)
    : BT::DecoratorNode(name, conf)
{
}

BT::PortsList ForEachMineralSampleNode::providedPorts()
{
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>>("samples_list"),
        BT::OutputPort<geometry_msgs::msg::PoseWithCovarianceStamped>("current_sample")
    };
}

void ForEachMineralSampleNode::halt()
{
    Reset();
    BT::DecoratorNode::halt();
}

BT::NodeStatus ForEachMineralSampleNode::tick()
{
    if(idle_) {
        BT::Optional<std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>> samples_port = getInput<std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>>("samples_list");
        if(!samples_port) {
            throw BT::RuntimeError("Missing required input [samples_list]: ", samples_port.error());
        }
        samples_ = samples_port.value();
        samples_iter_ = samples_.begin();
        idle_ = false;
        if(samples_.empty())
        {
            Reset();
            return BT::NodeStatus::SUCCESS;
        }
        setOutput("current_sample", *samples_iter_);
    }

    const auto child_status = child_node_->executeTick();

    switch(child_status)
    {
        case BT::NodeStatus::SUCCESS:
            ++samples_iter_;
            if(samples_iter_ == samples_.end())
            {
                Reset();
                return BT::NodeStatus::SUCCESS;
            }
            setOutput("current_sample", *samples_iter_);
            return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;
        default:
            Reset();
            return BT::NodeStatus::FAILURE;
    }
}

void ForEachMineralSampleNode::Reset()
{
    idle_ = true;
    samples_.clear();
    samples_iter_ = std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>::iterator{};
}

}  // namespace stsl_bt_nodes
