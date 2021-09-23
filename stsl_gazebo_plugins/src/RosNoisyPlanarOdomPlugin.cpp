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

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <random>
#include <string>

namespace stsl_gazebo_plugins
{

class RosNoisyPlanarOdomPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf) override
  {
    parent_ = parent;
    world_ = parent->GetWorld();

    odom_frame_ = sdf->Get<std::string>("odometry_frame", "odom").first;
    robot_frame_ = sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

    const auto publish_rate = sdf->Get<double>("publish_rate", 20.0).first;
    publish_period_sec_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;
    prev_publish_time_ = world_->SimTime();

    const auto vel_noise_std_dev = sdf->Get<double>("velocity_noise_std_dev", 0.25).first;
    random_engine_ = std::default_random_engine(std::random_device{} ());
    noise_distribution_ = std::normal_distribution<double>(0.0, vel_noise_std_dev);

    pub_covariance_x_ = sdf->Get<double>("covariance_x", 1e-4).first;
    pub_covariance_y_ = sdf->Get<double>("covariance_y", 1e-4).first;
    pub_covariance_yaw_ = sdf->Get<double>("covariance_yaw", 1e-2).first;

    gazebo_node_ = boost::make_shared<gazebo::transport::Node>();
    gazebo_node_->Init();

    ros_node_ = gazebo_ros::Node::Get(sdf);

    odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "~/odom",
      rclcpp::SystemDefaultsQoS());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    prev_update_time_ = world_->SimTime();

    update_connection_ =
      gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(
        &RosNoisyPlanarOdomPlugin::OnUpdate, this,
        std::placeholders::_1));
  }

  void Reset() override
  {
    prev_update_time_ = world_->SimTime();
    odom_pose_ = parent_->WorldPose();
    prev_odom_pose_.Reset();
    odom_linear_vel_ = ignition::math::Vector3d::Zero;
    odom_angular_vel_ = ignition::math::Vector3d::Zero;
  }

private:
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::transport::NodePtr gazebo_node_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::ModelPtr parent_;

  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::default_random_engine random_engine_;
  std::normal_distribution<double> noise_distribution_;

  ignition::math::Pose3d odom_pose_;
  ignition::math::Pose3d prev_odom_pose_;
  ignition::math::Vector3d odom_linear_vel_;
  ignition::math::Vector3d odom_angular_vel_;

  gazebo::common::Time prev_update_time_;

  double publish_period_sec_{10.0};
  gazebo::common::Time prev_publish_time_;

  std::string odom_frame_;
  std::string robot_frame_;

  double pub_covariance_x_;
  double pub_covariance_y_;
  double pub_covariance_yaw_;

  const double low_velocity_threshold_{1e-4};

  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    UpdateOdometry(info.simTime);

    if ( (info.simTime - prev_publish_time_).Double() >= publish_period_sec_) {
      prev_publish_time_ = info.simTime;
      PublishOdometry(info.simTime);
      BroadcastTF(info.simTime);
    }
  }

  void UpdateOdometry(const gazebo::common::Time & current_time)
  {
    prev_odom_pose_ = odom_pose_;

    const auto delta_time = (current_time - prev_update_time_).Double();
    prev_update_time_ = current_time;

    const auto & world_pose = parent_->WorldPose();

    auto relative_linear_vel = parent_->RelativeLinearVel();
    if (std::abs(relative_linear_vel.X()) > low_velocity_threshold_) {
      relative_linear_vel.X() += noise_distribution_(random_engine_);
    }
    if (std::abs(relative_linear_vel.Y()) > low_velocity_threshold_) {
      relative_linear_vel.Y() += noise_distribution_(random_engine_);
    }

    auto relative_angular_vel = parent_->RelativeAngularVel();
    if (std::abs(relative_angular_vel.Z()) > low_velocity_threshold_) {
      relative_angular_vel.Z() += noise_distribution_(random_engine_);
    }

    const auto rot_inv = world_pose.Rot().Inverse();
    odom_linear_vel_ = world_pose.Rot() * relative_linear_vel;
    odom_angular_vel_ = relative_angular_vel;

    odom_pose_.Pos() += odom_linear_vel_ * delta_time;

    odom_pose_.Rot() *= ignition::math::Quaternion(odom_angular_vel_ * delta_time);
  }

  void BroadcastTF(const gazebo::common::Time & current_time)
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = robot_frame_;
    msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_pose_);
    tf_broadcaster_->sendTransform(msg);
  }

  void PublishOdometry(const gazebo::common::Time & current_time)
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(current_time);
    msg.header.frame_id = odom_frame_;
    msg.child_frame_id = robot_frame_;
    msg.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(odom_pose_);
    msg.pose.covariance[0] = pub_covariance_x_;
    msg.pose.covariance[7] = pub_covariance_y_;
    msg.pose.covariance[14] = 1e12;
    msg.pose.covariance[21] = 1e12;
    msg.pose.covariance[28] = 1e12;
    msg.pose.covariance[35] = pub_covariance_yaw_;
    msg.twist.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_linear_vel_);
    msg.twist.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(odom_angular_vel_);
    msg.twist.covariance[0] = pub_covariance_x_;
    msg.twist.covariance[7] = pub_covariance_y_;
    msg.twist.covariance[14] = 1e12;
    msg.twist.covariance[21] = 1e12;
    msg.twist.covariance[28] = 1e12;
    msg.twist.covariance[35] = pub_covariance_yaw_;
    odom_pub_->publish(msg);
  }
};

GZ_REGISTER_MODEL_PLUGIN(RosNoisyPlanarOdomPlugin)

}  // namespace stsl_gazebo_plugins
