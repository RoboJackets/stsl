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
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <memory>

#include "src/motor.hpp"
#include "src/line_sensor.hpp"
#include "src/encoder.hpp"
#include "src/ultrasonic_sensor.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotInterfaceNode : public rclcpp::Node
{
public:
  RobotInterfaceNode()
  : Node("robot_interface_node")
  {
    left_encoder_publisher_ = create_publisher<std_msgs::msg::UInt64>("encoder_left", 1);
    right_encoder_publisher_ = create_publisher<std_msgs::msg::UInt64>("encoder_right", 1);
    center_line_publisher_ = create_publisher<std_msgs::msg::UInt64>("line_center", 1);
    side_line_publisher_ = create_publisher<std_msgs::msg::UInt64>("line_side", 1);
    ultrasonic_publisher_ = create_publisher<std_msgs::msg::Float32>("ultrasonic", 1);

    left_motor_subscriber_ = create_subscription<std_msgs::msg::Float32>(
      "motor_left", 1, std::bind(&RobotInterfaceNode::leftMotorCallback, this, _1));
    right_motor_subscriber_ = create_subscription<std_msgs::msg::Float32>(
      "motor_right", 1, std::bind(&RobotInterfaceNode::rightMotorCallback, this, _1));

    periodic_publishing_timer_ = create_wall_timer(
      100ms, std::bind(&RobotInterfaceNode::periodPublishingTimerCallback, this));

    RCLCPP_DEBUG(this->get_logger(), "Robot interface node ready!");
  }

private:
  void leftMotorCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Motor command: LEFT " + std::to_string(msg->data));
    left_motor_.setPower(msg->data);
  }

  void rightMotorCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Motor command: RIGHT " + std::to_string(msg->data));
    right_motor_.setPower(msg->data);
  }

  void periodPublishingTimerCallback()
  {
    RCLCPP_DEBUG(this->get_logger(), "Publishing periodic sensor measurements.");
    publishLineSensors();
    publishEncoders();
    ultrasonic_sensor_.trigger();
  }

  void publishLineSensors()
  {
    std_msgs::msg::UInt64 center_msg;
    center_msg.data = center_line_sensor_.getValue();
    center_line_publisher_->publish(center_msg);

    std_msgs::msg::UInt64 side_msg;
    side_msg.data = side_line_sensor_.getValue();
    side_line_publisher_->publish(side_msg);
  }

  void publishEncoders()
  {
    std_msgs::msg::UInt64 left_msg;
    left_msg.data = left_encoder_.getPosition();
    left_encoder_publisher_->publish(left_msg);

    std_msgs::msg::UInt64 right_msg;
    right_msg.data = right_encoder_.getPosition();
    right_encoder_publisher_->publish(right_msg);
  }

  void publishUltrasonic(float distance)
  {
    std_msgs::msg::Float32 msg;
    msg.data = distance;
    ultrasonic_publisher_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr left_encoder_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr right_encoder_publisher_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr center_line_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr side_line_publisher_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ultrasonic_publisher_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_motor_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_motor_subscriber_;

  rclcpp::TimerBase::SharedPtr periodic_publishing_timer_;

  Motor left_motor_{left_motor_parameters};
  Motor right_motor_{right_motor_parameters};

  LineSensor center_line_sensor_{1};
  LineSensor side_line_sensor_{3};

  Encoder left_encoder_{"GPMC_AD13"};
  Encoder right_encoder_{"GPMC_AD12"};

  UltrasonicSensor ultrasonic_sensor_{"GPMC_AD15", "GPMC_AD11",
    [this](float d) {publishUltrasonic(d);}};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
