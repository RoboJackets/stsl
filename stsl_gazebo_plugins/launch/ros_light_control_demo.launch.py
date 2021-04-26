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

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    test_world_path = os.sep.join([get_package_share_directory(
        'stsl_gazebo_plugins'), 'worlds', 'ros_light_control_demo_world.sdf'])
    gazebo_launch_path = os.sep.join(
        [get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'])

    stsl_gazebo_plugin_path = os.sep.join(
        [get_package_prefix('stsl_gazebo_plugins'), 'lib'])
    env_gazebo_plugin_path = os.pathsep.join(
        [stsl_gazebo_plugin_path, os.getenv('GAZEBO_PLUGIN_PATH', '')])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', env_gazebo_plugin_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world': test_world_path,
                'verbose': 'true',
                'gui_required': 'true'
            }.items()
        )
    ])
