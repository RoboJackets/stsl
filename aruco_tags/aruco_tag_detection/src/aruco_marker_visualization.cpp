#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace aruco_tag_detection
{
class ArucoVisualization : public rclcpp::Node
{
public:
  ArucoVisualization(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aruco_tag_visualization", options)
  {
    timer_ = create_wall_timer(
            0.1s, std::bind(&ArucoVisualization::PublishMarkers, this));
    QueryMarkers();

    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>("true_tags", 1);
    tag_sub_ = this->create_subscription<stsl_interfaces::msg::TagArray>("/aruco_tag_detector/tags",
               10, std::bind(&ArucoVisualization::tagCallback, this, std::placeholders::_1));
  }
private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;
  visualization_msgs::msg::Marker marker_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool publish_ = false;

  void tagCallback(const stsl_interfaces::msg::TagArray::SharedPtr tag_array_msg) {
    publish_ = true;
  }

  void QueryMarkers() {
    marker_msg_.header.frame_id = "/odom";
    marker_msg_.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;

    marker_msg_.pose.orientation.x = 0;
    marker_msg_.pose.orientation.y = 0;
    marker_msg_.pose.orientation.z = -0.7071068;
    marker_msg_.pose.orientation.w = 0.7071068;

    marker_msg_.scale.x = 0.3;
    marker_msg_.scale.y = 0.2;
    marker_msg_.scale.z = 0.1;

    marker_msg_.color.r = 1.0;
    marker_msg_.color.b = 1.0;
    marker_msg_.color.g = 1.0;

    for(int i = 0; i < 10; i++) {
      geometry_msgs::msg::Point point;
      point.x = 1.0*i;
      point.y = 0;
      point.z = 0;
      marker_msg_.points.push_back(point);
    }
  }

  void PublishMarkers() {
    if(!publish_)
    {
      return;
    }
    marker_msg_.header.stamp = this->now();

    marker_publisher_->publish(marker_msg_);
    publish_ = false;
  }
};
}  // namespace aruco_tag_detection

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  rclcpp::spin(std::make_shared<aruco_tag_detection::ArucoVisualization>(options));
  rclcpp::shutdown();
  return 0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_tag_detection::ArucoVisualization)
