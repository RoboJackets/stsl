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
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/aruco.hpp>

namespace aruco_tag_detection
{
class DetectorComponent : public rclcpp::Node
{
public:
  explicit DetectorComponent(const rclcpp::NodeOptions & options)
  : rclcpp::Node("aruco_tag_detector", options)
  {
    subscriber_ = image_transport::create_camera_subscription(this, "/camera/image_raw", std::bind(&DetectorComponent::ImageCallback, this, std::placeholders::_1, std::placeholders::_2), "raw", rmw_qos_profile_sensor_data);
    debug_publisher_ = image_transport::create_publisher(this, "~/debug_image");
  }

private:
  image_transport::CameraSubscriber subscriber_;
  image_transport::Publisher debug_publisher_;

  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg)
  {
    const auto cv_image = cv_bridge::toCvShare(image_msg, "bgr8");
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<std::vector<cv::Point2f>> rejected_candidates;
    auto detector_parameters = cv::aruco::DetectorParameters::create();
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(cv_image->image, dictionary, marker_corners, marker_ids, detector_parameters, rejected_candidates);

    // TODO publish marker poses

    if(debug_publisher_.getNumSubscribers() > 0)
    {
      auto debug_image = cv_image->image.clone();
      cv::aruco::drawDetectedMarkers(debug_image, marker_corners, marker_ids);
      cv_bridge::CvImage cv_debug_image(image_msg->header, "bgr8", debug_image);
      debug_publisher_.publish(cv_debug_image.toImageMsg());
    }
  }
};
}  // namespace aruco_tag_detection

RCLCPP_COMPONENTS_REGISTER_NODE(aruco_tag_detection::DetectorComponent)
