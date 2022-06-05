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

#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>

namespace stsl_aruco_tags
{
class DetectorComponent : public rclcpp::Node
{
public:
  explicit DetectorComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aruco_tag_detector", options)
  {
    tag_publisher_ = create_publisher<stsl_interfaces::msg::TagArray>("~/tags", 1);
    subscriber_ = image_transport::create_camera_subscription(
      this, "/camera/image_raw",
      std::bind(
        &DetectorComponent::ImageCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      "raw", rmw_qos_profile_sensor_data);
    debug_publisher_ = image_transport::create_publisher(this, "~/debug_image");
    marker_size_ = declare_parameter("tag_size", 0.2);
  }

private:
  rclcpp::Publisher<stsl_interfaces::msg::TagArray>::SharedPtr tag_publisher_;
  image_transport::CameraSubscriber subscriber_;
  image_transport::Publisher debug_publisher_;
  double marker_size_;

  void ImageCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & image_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
  {
    const auto cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    auto detector_parameters = cv::aruco::DetectorParameters::create();
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(
      cv_image->image, dictionary, marker_corners, marker_ids,
      detector_parameters, rejected_candidates);

    const auto camera_matrix = cv::Mat(info_msg->k).reshape(1, 3);
    const auto distortion_coefs = cv::Mat(info_msg->d).reshape(1, 5);

    std::vector<cv::Vec3d> rotations;
    std::vector<cv::Vec3d> translations;
    cv::aruco::estimatePoseSingleMarkers(
      marker_corners, marker_size_, camera_matrix,
      distortion_coefs, rotations, translations);

    if (tag_publisher_->get_subscription_count() > 0) {
      stsl_interfaces::msg::TagArray tag_array_msg;
      tag_array_msg.header.frame_id = image_msg->header.frame_id;
      tag_array_msg.header.stamp = image_msg->header.stamp;
      const auto tag_count = marker_ids.size();
      for (auto tag_index = 0; tag_index < tag_count; tag_index++) {
        stsl_interfaces::msg::Tag tag_msg;
        tag_msg.id = marker_ids[tag_index];
        tag_msg.pose.position.x = translations[tag_index][0];
        tag_msg.pose.position.y = translations[tag_index][1];
        tag_msg.pose.position.z = translations[tag_index][2];

        const auto rvec = Eigen::Vector3d(
          rotations[tag_index][0], rotations[tag_index][1],
          rotations[tag_index][2]);
        Eigen::AngleAxisd angle_axis(rvec.norm(), rvec.normalized());
        Eigen::Quaterniond quaternion(angle_axis);
        tag_msg.pose.orientation.w = quaternion.w();
        tag_msg.pose.orientation.x = quaternion.x();
        tag_msg.pose.orientation.y = quaternion.y();
        tag_msg.pose.orientation.z = quaternion.z();
        tag_array_msg.tags.push_back(tag_msg);
      }
      tag_publisher_->publish(tag_array_msg);
    }

    if (debug_publisher_.getNumSubscribers() > 0) {
      auto debug_image = cv_image->image.clone();
      cv::aruco::drawDetectedMarkers(debug_image, marker_corners, marker_ids);
      for (int tag_index = 0; tag_index < marker_corners.size(); tag_index++) {
        cv::aruco::drawAxis(
          debug_image, camera_matrix, distortion_coefs, rotations[tag_index],
          translations[tag_index], 0.1);
      }
      cv_bridge::CvImage cv_debug_image(image_msg->header, "bgr8", debug_image);
      debug_publisher_.publish(cv_debug_image.toImageMsg());
    }
  }
};
}  // namespace stsl_aruco_tags

RCLCPP_COMPONENTS_REGISTER_NODE(stsl_aruco_tags::DetectorComponent)
