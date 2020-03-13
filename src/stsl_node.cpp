#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class StslNode : public rclcpp::Node
{
public:
    StslNode() : Node("stsl_node")
    {
        publisher_ = create_publisher<std_msgs::msg::String>("topic_out", 10);
        subscription_ = create_subscription<std_msgs::msg::String>("topic_in", 10, std::bind(&StslNode::callback, this, _1));
    }

private:
    void callback(const std_msgs::msg::String::SharedPtr message_in) const
    {
        RCLCPP_INFO(get_logger(), "Message Received: '%s'", message_in->data.c_str());
        
        auto message_out = std_msgs::msg::String();
        message_out.data = message_in->data;
        std::reverse(message_out.data.begin(), message_out.data.end());
        
        RCLCPP_INFO(get_logger(), "Sending Message: '%s'", message_out.data.c_str()); 
        
        publisher_->publish(message_out);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StslNode>());
    rclcpp::shutdown();
    return 0;
}

