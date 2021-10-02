#! /usr/bin/python3

# Copyright 2021 RoboJackets
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import rclpy
from rclpy.qos import DurabilityPolicy, qos_profile_system_default
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Point32


class BoundaryPublisher(Node):
    def __init__(self):
        super().__init__('boundary_publisher')
        qos = qos_profile_system_default
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(
            PolygonStamped, '/map_boundary', qos)
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
