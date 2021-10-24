#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace stsl_utils
{

class FloorMatVisualizer : public rclcpp::Node
{
public:
  explicit FloorMatVisualizer(const rclcpp::NodeOptions& options)
    : rclcpp::Node("floor_mat_visualizer", options)
  {
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("~/marker", rclcpp::SystemDefaultsQoS());
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&FloorMatVisualizer::TimerCallback, this));
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void TimerCallback() {
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.ns = "/floor_mat_visualizer";
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.lifetime = rclcpp::Duration(std::chrono::seconds(1));
    msg.text = "SHOW ME!";
    msg.frame_locked = true;
    msg.scale.x = 1;
    msg.scale.y = 1;
    msg.scale.z = 1;
    msg.mesh_resource = "package://stsl_utils/visualization_meshes/floor_mat/floor_mat.dae";
    msg.mesh_use_embedded_materials = true;
    marker_pub_->publish(msg);
  }

};

}  // namespace stsl_utils

RCLCPP_COMPONENTS_REGISTER_NODE(stsl_utils::FloorMatVisualizer)
