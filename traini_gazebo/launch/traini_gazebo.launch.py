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
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler,
                            ExecuteProcess)
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    world_path = os.sep.join([get_package_share_directory(
        'traini_gazebo'), 'worlds', 'training_world.sdf'])
    gazebo_launch_path = os.sep.join(
        [get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'])

    traini_gazebo_model_path = os.sep.join(
        [get_package_share_directory('traini_gazebo'), 'models'])
    gazebo_model_path = os.pathsep.join(
        [traini_gazebo_model_path, os.getenv('GAZEBO_MODEL_PATH', '')])

    stsl_gazebo_plugin_path = os.sep.join(
        [get_package_prefix('stsl_gazebo_plugins'), 'lib'])
    gazebo_plugin_path = os.pathsep.join(
        [stsl_gazebo_plugin_path, os.getenv('GAZEBO_PLUGIN_PATH', '')])

    kill_server_script_path = os.sep.join(
        [get_package_prefix('traini_gazebo'), 'bin', 'kill_gazebo_server.sh'])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        # Disable trying to connect to online model database
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world': world_path,
                'verbose': 'false',
                'gui_required': 'true',
                'server_required': 'false',
                'factory': 'false'
            }.items()
        ),
        RegisterEventHandler(
            launch.event_handlers.OnShutdown(
                # This is a workaround for this issue:
                # https://github.com/ros-simulation/gazebo_ros_pkgs/issues/1198
                on_shutdown=[ExecuteProcess(cmd=[kill_server_script_path])]
            )
        )
    ])
