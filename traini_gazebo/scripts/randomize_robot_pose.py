#! /usr/bin/python3

import random
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState


class RandomizeRobotPoseNode(Node):

    def __init__(self):
        super().__init__('randomize_robot_pose')
        self.client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = SetEntityState.Request()

    def send_request(self):
        self.request.state.name = 'Traini'
        self.request.state.pose.position.x = random.uniform(-0.3, 0.3)
        self.request.state.pose.position.y = random.uniform(-0.15, 0.15)
        self.future = self.client.call_async(self.request)

    def check_response(self):
        if self.future.done():
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().error(f'Service call failed {e}')
                return True
            else:
                if response:
                    self.get_logger().info('Done!')
                    return True
                else:
                    self.get_logger().info("Simulator couldn't move the robot!")
                    return True
        else:
            return False


def main(args=None):
    rclpy.init(args=args)

    node = RandomizeRobotPoseNode()
    node.send_request()

    while rclpy.ok():
        rclpy.spin_once(node)
        if node.check_response():
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
