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

#ifndef YAML_HELPERS_HPP_
#define YAML_HELPERS_HPP_

#include <yaml-cpp/yaml.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

namespace mission_orchestration::yaml_helpers
{

template<typename MessageType>
MessageType fromYaml(const YAML::Node & yaml);


template<>
std::array<double, 36> fromYaml(const YAML::Node & yaml);

template<>
builtin_interfaces::msg::Time fromYaml(const YAML::Node & yaml);

template<>
geometry_msgs::msg::Point fromYaml(const YAML::Node & yaml);

template<>
geometry_msgs::msg::Quaternion fromYaml(const YAML::Node & yaml);

template<>
std_msgs::msg::Header fromYaml(const YAML::Node & yaml);

template<>
geometry_msgs::msg::Pose fromYaml(const YAML::Node & yaml);

template<>
geometry_msgs::msg::PoseWithCovariance fromYaml(const YAML::Node & yaml);

template<>
geometry_msgs::msg::PoseWithCovarianceStamped fromYaml(const YAML::Node & yaml);

}  // namespace mission_orchestration::yaml_helpers

#endif  // YAML_HELPERS_HPP_
