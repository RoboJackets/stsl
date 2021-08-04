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

using namespace std::chrono_literals;

namespace stsl_aruco_tags
{
class ArucoVisualization : public rclcpp::Node
{
public:
  explicit ArucoVisualization(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aruco_tag_visualization", options)
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("true_tags", 1);
    tag_sub_ = this->create_subscription<stsl_interfaces::msg::TagArray>(
      "/aruco_tag_detector/tags",
      10, std::bind(&ArucoVisualization::tagCallback, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;

  void tagCallback(const stsl_interfaces::msg::TagArray::SharedPtr tag_array_msg)
  {
    visualization_msgs::msg::MarkerArray marker_array_msg;

    for (const stsl_interfaces::msg::Tag & tag : tag_array_msg->tags) {
      visualization_msgs::msg::Marker marker_msg;
      marker_msg.id = tag.id;
      marker_msg.header = tag_array_msg->header;
      marker_msg.type = visualization_msgs::msg::Marker::CUBE;
      marker_msg.action = visualization_msgs::msg::Marker::ADD;
      marker_msg.ns = "/tag_visualization";
      marker_msg.lifetime = rclcpp::Duration(0.15s);

      marker_msg.pose.position.x = tag.pose.position.x;
      marker_msg.pose.position.y = tag.pose.position.y;
      marker_msg.pose.position.z = tag.pose.position.z;

      marker_msg.pose.orientation.x = tag.pose.orientation.x;
      marker_msg.pose.orientation.y = tag.pose.orientation.y;
      marker_msg.pose.orientation.z = tag.pose.orientation.z;
      marker_msg.pose.orientation.w = tag.pose.orientation.w;

      marker_msg.color.a = 0.5;
      marker_msg.color.r = 0.0;
      marker_msg.color.b = 0.0;
      marker_msg.color.g = 1.0;

      marker_msg.scale.x = 0.18;
      marker_msg.scale.y = 0.18;
      marker_msg.scale.z = 0.02;

      marker_array_msg.markers.push_back(marker_msg);
    }

    marker_pub_->publish(marker_array_msg);
  }
};
}  // namespace stsl_aruco_tags

RCLCPP_COMPONENTS_REGISTER_NODE(stsl_aruco_tags::ArucoVisualization)
