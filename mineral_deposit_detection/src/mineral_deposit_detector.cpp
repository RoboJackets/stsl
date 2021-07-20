#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <stsl_interfaces/msg/tag_array.hpp>
#include <stsl_interfaces/msg/mineral_deposit_array.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>

namespace mineral_deposit_detection
{

class MineralDepositDetector : public rclcpp::Node
{
public:
  explicit MineralDepositDetector(const rclcpp::NodeOptions & options)
  : rclcpp::Node("mineral_deposit_detector", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    deposit_ids_(declare_parameter<std::vector<int64_t>>("deposit_tag_ids", {})),
    random_engine_(std::random_device{}()),
    range_noise_distribution_(0.0, declare_parameter<double>("range_noise_stddev", 0.0)),
    heading_noise_distribution_(0.0, declare_parameter<double>("heading_noise_stddev", 0.0))
  {
    deposit_pub_ = create_publisher<stsl_interfaces::msg::MineralDepositArray>(
      "~/deposits",
      rclcpp::SystemDefaultsQoS());
    tag_sub_ = create_subscription<stsl_interfaces::msg::TagArray>(
      "~/tags",
      rclcpp::SensorDataQoS(),
      std::bind(&MineralDepositDetector::TagCallback, this, std::placeholders::_1));
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  const std::vector<int64_t> deposit_ids_;
  std::default_random_engine random_engine_;
  std::normal_distribution<double> range_noise_distribution_;
  std::normal_distribution<double> heading_noise_distribution_;
  rclcpp::Publisher<stsl_interfaces::msg::MineralDepositArray>::SharedPtr deposit_pub_;
  rclcpp::Subscription<stsl_interfaces::msg::TagArray>::SharedPtr tag_sub_;

  void TagCallback(const stsl_interfaces::msg::TagArray::ConstSharedPtr tag_msg)
  {
    try {
      stsl_interfaces::msg::MineralDepositArray deposit_msg;

      deposit_msg.header = tag_msg->header;

      stsl_interfaces::msg::TagArray::_tags_type deposit_tags;
      std::copy_if(tag_msg->tags.begin(), tag_msg->tags.end(), std::back_inserter(deposit_tags), [this](const auto& tag){
        return IsDepositTag(tag);
      });

      const auto transform = tf_buffer_.lookupTransform("base_link", tag_msg->header.frame_id, tag_msg->header.stamp);

      std::transform(
        deposit_tags.begin(), deposit_tags.end(), std::back_inserter(deposit_msg.deposits),
        [this, &transform](const auto & tag) {
          return TagToDeposit(tag, transform);
        });

      deposit_pub_->publish(deposit_msg);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1, "TF Error: %s", e.what());
    }
  }

  bool IsDepositTag(const stsl_interfaces::msg::Tag & tag)
  {
    return std::find(deposit_ids_.begin(), deposit_ids_.end(), tag.id) != deposit_ids_.end();
  }

  stsl_interfaces::msg::MineralDeposit TagToDeposit(const stsl_interfaces::msg::Tag & tag, const geometry_msgs::msg::TransformStamped & transform)
  {
    stsl_interfaces::msg::MineralDeposit deposit_msg;
    deposit_msg.id = tag.id;

    geometry_msgs::msg::PoseStamped base_link_pose;
    geometry_msgs::msg::PoseStamped tag_pose_stamped;
    tag_pose_stamped.pose = tag.pose;

    tf2::doTransform(tag_pose_stamped, base_link_pose, transform);

    deposit_msg.range = std::hypot(base_link_pose.pose.position.x, base_link_pose.pose.position.y);
    deposit_msg.heading = std::atan2(base_link_pose.pose.position.y, base_link_pose.pose.position.x);

    deposit_msg.range += range_noise_distribution_(random_engine_);
    deposit_msg.heading += heading_noise_distribution_(random_engine_);

    return deposit_msg;
  }
};

}  // namespace mineral_deposit_detection

RCLCPP_COMPONENTS_REGISTER_NODE(mineral_deposit_detection::MineralDepositDetector)
