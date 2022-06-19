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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def opaque_launch_function(context, *args, **kwargs):
    world_path = os.sep.join([get_package_share_directory('traini_ign_gazebo'), 'worlds', 'training_world.sdf'])
    gazebo_launch_path = os.sep.join([get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'])

    models_path = os.sep.join([get_package_share_directory('traini_ign_gazebo'), 'models'])
    ign_gazebo_resource_path = os.pathsep.join([os.getenv('IGN_GAZEBO_RESOURCE_PATH', ''), models_path])

    headless_flag = '-s' if LaunchConfiguration('headless').perform(context) == 'true' else ''

    auto_start_flag = '-r' if LaunchConfiguration('auto_start').perform(context) == 'true' else ''

    render_engine_arg = LaunchConfiguration('render_engine').perform(context)
    if render_engine_arg != '':
        render_engine_flag = '--render-engine ' + render_engine_arg
    else:
        render_engine_flag = ''

    ign_args = ' '.join([
        world_path,
        headless_flag,
        auto_start_flag,
        render_engine_flag
    ])

    return [
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ign_gazebo_resource_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'ign_args': ign_args
            }.items()
        )
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value='ogre'),
        OpaqueFunction(function=opaque_launch_function),
    ])
