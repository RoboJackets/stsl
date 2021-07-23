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

#include "yaml_helpers.hpp"
#include <string>

namespace mission_orchestration::yaml_helpers
{

void assertHasMember(const YAML::Node & yaml, const std::string & name)
{
  if (!yaml[name]) {
    throw YAML::ParserException(yaml.Mark(), "Missing member: " + name);
  }
}

void assertIsMap(const YAML::Node & yaml, const std::string & message_type)
{
  if (!yaml.IsMap()) {
    throw YAML::ParserException(yaml.Mark(), message_type + " must be parsed from a map.");
  }
}

template<>
std::array<double, 36> fromYaml(const YAML::Node & yaml)
{
  if (!yaml.IsSequence()) {
    throw YAML::ParserException(
            yaml.Mark(), "std::array<double, 36> must be parsed from a sequence.");
  }

  if (yaml.size() != 36) {
    throw YAML::ParserException(
            yaml.Mark(),
            "Expected 36 elements in sequence, but got " + std::to_string(yaml.size()));
  }

  std::array<double, 36> msg;

  std::transform(
    yaml.begin(), yaml.end(), msg.begin(), [](const YAML::Node & n) {
      return n.as<double>();
    });

  return msg;
}

template<>
geometry_msgs::msg::Point fromYaml(const YAML::Node & yaml)
{
  assertIsMap(yaml, "geometry_msgs::msg::Point");

  geometry_msgs::msg::Point msg;

  assertHasMember(yaml, "x");
  msg.x = yaml["x"].as<double>();

  assertHasMember(yaml, "y");
  msg.y = yaml["y"].as<double>();

  assertHasMember(yaml, "z");
  msg.z = yaml["z"].as<double>();

  return msg;
}

template<>
geometry_msgs::msg::Quaternion fromYaml(const YAML::Node & yaml)
{
  assertIsMap(yaml, "geometry_msgs::msg::Quaternion");

  geometry_msgs::msg::Quaternion msg;

  assertHasMember(yaml, "x");
  msg.x = yaml["x"].as<double>();

  assertHasMember(yaml, "y");
  msg.y = yaml["y"].as<double>();

  assertHasMember(yaml, "z");
  msg.z = yaml["z"].as<double>();

  assertHasMember(yaml, "w");
  msg.w = yaml["w"].as<double>();

  return msg;
}

template<>
geometry_msgs::msg::Pose fromYaml(const YAML::Node & yaml)
{
  assertIsMap(yaml, "geometry_msgs::msg::Pose");

  geometry_msgs::msg::Pose msg;

  assertHasMember(yaml, "position");

  msg.position = fromYaml<geometry_msgs::msg::Point>(yaml["position"]);

  assertHasMember(yaml, "orientation");

  msg.orientation = fromYaml<geometry_msgs::msg::Quaternion>(yaml["orientation"]);

  return msg;
}

template<>
geometry_msgs::msg::PoseWithCovariance fromYaml(const YAML::Node & yaml)
{
  assertIsMap(yaml, "geometry_msgs::msg::PoseWithCovariance");

  geometry_msgs::msg::PoseWithCovariance msg;

  assertHasMember(yaml, "pose");

  msg.pose = fromYaml<geometry_msgs::msg::Pose>(yaml["pose"]);

  assertHasMember(yaml, "covariance");

  msg.covariance = fromYaml<std::array<double, 36>>(yaml["covariance"]);

  return msg;
}

template<>
stsl_interfaces::msg::MineralDepositSample fromYaml(const YAML::Node & yaml)
{
  assertIsMap(yaml, "stsl_interfaces::msg::MineralDepositSample");

  stsl_interfaces::msg::MineralDepositSample msg;

  assertHasMember(yaml, "id");
  msg.id = yaml["id"].as<int>();

  assertHasMember(yaml, "pose");
  msg.pose = fromYaml<geometry_msgs::msg::PoseWithCovariance>(yaml["pose"]);

  return msg;
}

}  // namespace mission_orchestration::yaml_helpers
