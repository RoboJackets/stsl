// Copyright 2021 RoboJackets
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <string>
#include <vector>

using namespace std::chrono_literals;

namespace stsl_aruco_tags
{
class ArucoMarkerVisualizer : public rclcpp::Node
{
public:
  explicit ArucoMarkerVisualizer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aruco_marker_visualizer", options)
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/markers", 1);
    tag_sub_ = this->create_subscription<stsl_interfaces::msg::TagArray>(
      "~/tags",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&ArucoMarkerVisualizer::tagCallback, this, std::placeholders::_1));
    declare_parameter<double>("marker_lifetime", 0.15);
    declare_parameter<double>("marker_size", 0.18);
    declare_parameter<double>("marker_thickness", 0.02);
    declare_parameter<std::vector<double>>("marker_color_rgba", {0.0, 0.0, 1.0, 0.5});
    declare_parameter<std::string>("marker_namespace", "tag_visualization");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;

  void tagCallback(const stsl_interfaces::msg::TagArray::SharedPtr tag_array_msg)
  {
    visualization_msgs::msg::MarkerArray marker_array_msg;

    std::transform(
      tag_array_msg->tags.begin(), tag_array_msg->tags.end(),
      std::back_inserter(marker_array_msg.markers),
      [this, & header = tag_array_msg->header](const auto & tag) {
        return tagToMarker(tag, header);
      });

    marker_pub_->publish(marker_array_msg);
  }

  visualization_msgs::msg::Marker tagToMarker(
    const stsl_interfaces::msg::Tag & tag,
    const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::Marker marker_msg;
    marker_msg.id = tag.id;
    marker_msg.header = header;
    marker_msg.type = visualization_msgs::msg::Marker::CUBE;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.ns = get_parameter("marker_namespace").as_string();
    marker_msg.lifetime =
      rclcpp::Duration(std::chrono::duration<double>(get_parameter("marker_lifetime").as_double()));

    marker_msg.pose.position.x = tag.pose.position.x;
    marker_msg.pose.position.y = tag.pose.position.y;
    marker_msg.pose.position.z = tag.pose.position.z;

    marker_msg.pose.orientation.x = tag.pose.orientation.x;
    marker_msg.pose.orientation.y = tag.pose.orientation.y;
    marker_msg.pose.orientation.z = tag.pose.orientation.z;
    marker_msg.pose.orientation.w = tag.pose.orientation.w;

    const auto color = get_parameter("marker_color").as_double_array();

    marker_msg.color.r = color.at(0);
    marker_msg.color.b = color.at(1);
    marker_msg.color.g = color.at(2);
    marker_msg.color.a = color.at(3);

    const auto size = get_parameter("marker_size").as_double();
    marker_msg.scale.x = size;
    marker_msg.scale.y = size;
    marker_msg.scale.z = get_parameter("marker_thickness").as_double();

    return marker_msg;
  }
};
}  // namespace stsl_aruco_tags

RCLCPP_COMPONENTS_REGISTER_NODE(stsl_aruco_tags::ArucoMarkerVisualizer)
