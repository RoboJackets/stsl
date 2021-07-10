#ifndef MISSION_ORCHESTRATION__YAML_HELPERS_HPP_
#define MISSION_ORCHESTRATION__YAML_HELPERS_HPP_

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

}

#endif  // MISSION_ORCHESTRATION__YAML_HELPERS_HPP_