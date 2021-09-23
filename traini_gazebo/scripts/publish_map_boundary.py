#! /usr/bin/python3

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32

class BoundaryPublisher(Node):
  def __init__(self):
    super().__init__('boundary_publisher')
    qos = qos_profile_system_default
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    self.publisher = self.create_publisher(PolygonStamped, '/map_boundary', qos)
    self.publish_boundary()
  
  def publish_boundary(self):
    msg = PolygonStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = self.get_clock().now().to_msg()
    map_width = 1.2192
    map_height = 0.762
    msg.polygon.points = [
      Point32(x=map_width/2, y=map_height/2),
      Point32(x=map_width/2, y=-map_height/2),
      Point32(x=-map_width/2, y=-map_height/2),
      Point32(x=-map_width/2, y=map_height/2)
    ]
    self.publisher.publish(msg)

def main(args=None):
  rclpy.init(args=args)
  node = BoundaryPublisher()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
