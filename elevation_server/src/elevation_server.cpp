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
#include <stsl_interfaces/srv/sample_elevation.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <tf2_ros/transform_listener.h>
#include <string>
#include <memory>

namespace elevation_server
{
class ElevationServer : public rclcpp::Node
{
public:
  explicit ElevationServer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("elevation_server", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<double>("max_range", 0.1);
    declare_parameter<double>("map_origin.x", 0.0);
    declare_parameter<double>("map_origin.y", 0.0);
    declare_parameter<double>("map_resolution", 0.01);
    declare_parameter<double>("elevation_scale", 1.0);

    const auto map_path = declare_parameter<std::string>("map_file", "");
    if (map_path.empty()) {
      RCLCPP_ERROR(get_logger(), "Must specify map_file path");
      return;
    }
    cv::Mat map_image = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    if (map_image.empty()) {
      RCLCPP_ERROR(get_logger(), "Could not load map from path: '%s'", map_path.c_str());
      return;
    }
    map_image.convertTo(map_, CV_64F, 1 / 255.0);

    service_ = create_service<stsl_interfaces::srv::SampleElevation>(
      "/sample_elevation", std::bind(
        &ElevationServer::ServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2));
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Service<stsl_interfaces::srv::SampleElevation>::SharedPtr service_;
  cv::Mat map_;

  void
  ServiceCallback(
    const std::shared_ptr<stsl_interfaces::srv::SampleElevation::Request> request,
    std::shared_ptr<stsl_interfaces::srv::SampleElevation::Response> response)
  {
    double robot_x;
    double robot_y;
    if (!GetRobotPosition(robot_x, robot_y)) {
      response->success = false;
      return;
    }
    const auto distance = std::hypot(robot_x - request->x, robot_y - request->y);
    if (distance > get_parameter("max_range").as_double()) {
      RCLCPP_ERROR(get_logger(), "Requested location out of range from robot's current position. Robot at <%f, %f>. Request at <%f, %f>.", robot_x, robot_y, request->x, request->y);
      response->success = false;
      return;
    }
    if(!IsInMap(request->x, request->y)) {
      RCLCPP_WARN_ONCE(get_logger(), "Requested location outside of map. Elevation will be -1. This message only prints once.");
      response->elevation = -1;
    } else {
      response->elevation = GetElevation(request->x, request->y);
    }
    response->success = true;
  }

  void MapCoordsFromMeters(const double meters_x, const double meters_y, int & map_x, int & map_y)
  {
    const auto origin_x = get_parameter("map_origin.x").as_double();
    const auto origin_y = get_parameter("map_origin.y").as_double();
    const auto resolution = get_parameter("map_resolution").as_double();
    map_x = (meters_x - origin_x) / resolution;
    map_y = (meters_y - origin_y) / resolution;
  }

  bool IsInMap(const double x, const double y)
  {
    int py;
    int px;
    MapCoordsFromMeters(x, y, px, py);
    return px > 0 && px < map_.cols && py > 0 && py < map_.rows;
  }

  double GetElevation(const double x, const double y)
  {
    const auto elevation_scale = get_parameter("elevation_scale").as_double();
    int px;
    int py;
    MapCoordsFromMeters(x, y, px, py);
    return map_.at<double>(py, px) * elevation_scale;
  }

  bool GetRobotPosition(double & x, double & y)
  {
    std::string tf_error_;
    if (!tf_buffer_.canTransform("map", "base_footprint", tf2::TimePointZero, &tf_error_)) {
      RCLCPP_ERROR(
        get_logger(), "Could not lookup transform 'map' -> 'base_footprint': %s",
        tf_error_.c_str());
      return false;
    }
    const auto transform = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  }
};

}  // namespace elevation_server

RCLCPP_COMPONENTS_REGISTER_NODE(elevation_server::ElevationServer)
