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

"""
Starts up nodes needed to drive the robot with a gamepad/joystick.

Normal usage, with default configs for the classroom joysticks:
$ ros2 launch traini_bringup joystick_control.launch.py

To use with your own joystick, you may need to modify the gamepad settings.
Copy config/joystick_parameters.yaml somewhere, adjust the settings within, and use this new copy
$ ros2 launch traini_bringup joystick_control.launch.py config_file:=/path/to/your/config/file.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    default_parameters_file_path = os.path.join(get_package_share_directory(
        'traini_bringup'), 'config', 'joystick_params_playstation.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_path', default_value=default_parameters_file_path),
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
