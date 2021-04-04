import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    test_world_path=os.sep.join([get_package_share_directory('stsl_gazebo_plugins'),'worlds','ros_light_control_demo_world.sdf'])
    gazebo_launch_path=os.sep.join([get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'])

    stsl_gazebo_plugin_path=os.sep.join([get_package_prefix('stsl_gazebo_plugins'),'lib'])
    env_gazebo_plugin_path=os.pathsep.join([stsl_gazebo_plugin_path, os.getenv('GAZEBO_PLUGIN_PATH','')])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', env_gazebo_plugin_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world':test_world_path,
                'verbose':'true',
                'gui_required':'true'
                }.items()
        )
    ])
