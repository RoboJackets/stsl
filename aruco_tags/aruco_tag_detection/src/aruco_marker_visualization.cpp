#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
            1.0s, std::bind(&ArucoVisualization::PublishMarkers, this));
    QueryMarkers();

    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>("true_tags", 1);
    tag_sub_ = this->create_subscription<stsl_interfaces::msg::TagArray>("/aruco_tag_detector/tags",
               10, std::bind(&ArucoVisualization::tagCallback, this, std::placeholders::_1));
  }
private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;
  visualization_msgs::msg::MarkerArray marker_array_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool publish_ = false;

  void tagCallback(const stsl_interfaces::msg::TagArray::SharedPtr tag_array_msg) {
    publish_ = true;
  }

  void QueryMarkers() {

    for(int i = 0; i < 2; i++) {
      visualization_msgs::msg::Marker marker_msg;
      marker_msg.id = i;
      marker_msg.header.frame_id = "/odom";
      marker_msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;

      marker_msg.pose.orientation.x = 0.0;
      marker_msg.pose.orientation.y = 0.0;
      marker_msg.pose.orientation.z = 0.0;
      marker_msg.pose.orientation.w = 1.0;

      marker_msg.color.a = 0.5;
      marker_msg.color.r = 0.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.g = 1.0;

      marker_array_msg_.markers.push_back(marker_msg);
    }

    marker_array_msg_.markers[0].scale.x = 0.025;
    marker_array_msg_.markers[0].scale.y = 0.2;
    marker_array_msg_.markers[0].scale.z = 0.2;

    marker_array_msg_.markers[0].points.resize(2);
    marker_array_msg_.markers[0].points[0].x = 0.6096;
    marker_array_msg_.markers[0].points[0].y = 0;
    marker_array_msg_.markers[0].points[0].z = 0.05;
    marker_array_msg_.markers[0].points[1].x = -0.6096;
    marker_array_msg_.markers[0].points[1].y = 0;
    marker_array_msg_.markers[0].points[1].z = 0.05;

    marker_array_msg_.markers[1].scale.x = 0.2;
    marker_array_msg_.markers[1].scale.y = 0.025;
    marker_array_msg_.markers[1].scale.z = 0.2;

    marker_array_msg_.markers[1].points.resize(4);
    marker_array_msg_.markers[1].points[0].x = -0.3;
    marker_array_msg_.markers[1].points[0].y = -0.381;
    marker_array_msg_.markers[1].points[0].z = 0.05;
    marker_array_msg_.markers[1].points[1].x = 0.3;
    marker_array_msg_.markers[1].points[1].y = -0.381;
    marker_array_msg_.markers[1].points[1].z = 0.05;
    marker_array_msg_.markers[1].points[2].x = -0.3;
    marker_array_msg_.markers[1].points[2].y = 0.381;
    marker_array_msg_.markers[1].points[2].z = 0.05;
    marker_array_msg_.markers[1].points[3].x = 0.3;
    marker_array_msg_.markers[1].points[3].y = 0.381;
    marker_array_msg_.markers[1].points[3].z = 0.05;
  }

  void PublishMarkers() {
    if(!publish_)
    {
      return;
    }
    for(int i = 0; i < marker_array_msg_.markers.size(); i++) {
      marker_array_msg_.markers[i].header.stamp = this->now();
      marker_array_msg_.markers[i].action = 0;
      marker_array_msg_.markers[i].lifetime = rclcpp::Duration(1.0s);
    }

    marker_publisher_->publish(marker_array_msg_);
    publish_ = false;
  }
};
}  // namespace aruco_tag_detection

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_tag_detection::ArucoVisualization)
