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

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix


def generate_launch_description():
    return LaunchDescription([
        AppendEnvironmentVariable('GAZEBO_MODEL_PATH', PathJoinSubstitution(
            [FindPackageShare('traini_gazebo'), 'models'])),
        AppendEnvironmentVariable('GAZEBO_PLUGIN_PATH', PathJoinSubstitution(
            [FindPackagePrefix('stsl_gazebo_plugins'), 'lib'])),
        # Disable trying to connect to online model database
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        DeclareLaunchArgument(
            'verbose', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('gui', default_value='true',
                              choices=['false', 'true']),
        IncludeLaunchDescription(
            # TODO(barulicm) Copying the gazebo_ros launch files into this directory is a
            # temporary way to get the params_file feature before it's been released
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('traini_gazebo'), 'launch', 'gazebo.launch.py'])),
            launch_arguments={
                'world': PathJoinSubstitution([FindPackageShare('traini_gazebo'),
                                               'worlds', 'training_world.sdf']),
                'verbose': LaunchConfiguration('verbose'),
                'gui': LaunchConfiguration('gui'),
                'gui_required': 'true',
                'server_required': 'false',
                'factory': 'false',
                'params_file': PathJoinSubstitution([FindPackageShare('traini_gazebo'),
                                                     'config', 'gazebo_params.yaml']),
            }.items()
        )
    ])
