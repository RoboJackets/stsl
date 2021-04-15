"""
Starts up nodes needed to drive the robot with a gamepad/joystick.

Normal usage, with default configs for the classroom joysticks:
$ ros2 launch traini_bringup joystick_control.launch.py

To use with your own joystick, you may need to modify the gamepad settings.
Copy config/joystick_parameters.yaml somewhere, adjust the settings within, and use this new copy
$ ros2 launch traini_bringup joystick_control.launch.py config_file:=/path/to/your/config/file.yaml
"""
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_parameters_file_path=os.path.join(get_package_share_directory('traini_bringup'),'config','joystick_parameters.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('config_path', default_value=default_parameters_file_path),
        LogInfo(
            msg=LaunchConfiguration('config_path')
        ),
        Node(
            package='joy',
            executable='joy_node',
            output='screen',
            parameters=[LaunchConfiguration('config_path')]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            output='screen',
            parameters=[LaunchConfiguration('config_path')]
        )
    ])