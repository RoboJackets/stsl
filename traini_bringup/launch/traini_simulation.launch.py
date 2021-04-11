import os
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    gazebo_launch_path=os.sep.join([get_package_share_directory('traini_gazebo'),'launch','traini_gazebo.launch.py'])
    robot_state_pub_path=os.sep.join([get_package_share_directory('traini_description'),'launch','robot_state_publisher.launch.py'])

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gazebo_launch_path)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_pub_path))
    ])
