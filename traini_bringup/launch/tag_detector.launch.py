import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='aruco_tag_detection',
            executable='aruco_tag_detection_node',
            output='screen'
        )
    ])
