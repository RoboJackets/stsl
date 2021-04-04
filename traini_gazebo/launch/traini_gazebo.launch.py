import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    world_path=os.sep.join([get_package_share_directory('traini_gazebo'),'worlds','training_world.sdf'])
    gazebo_launch_path=os.sep.join([get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'])

    traini_gazebo_model_path=os.sep.join([get_package_share_directory('traini_gazebo'),'models'])
    gazebo_model_path=os.pathsep.join([traini_gazebo_model_path, os.getenv('GAZEBO_MODEL_PATH','')])

    stsl_gazebo_plugin_path=os.sep.join([get_package_prefix('stsl_gazebo_plugins'),'lib'])
    gazebo_plugin_path=os.pathsep.join([stsl_gazebo_plugin_path, os.getenv('GAZEBO_PLUGIN_PATH','')])

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', gazebo_plugin_path),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', gazebo_model_path),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path),
            launch_arguments={
                'world':world_path,
                'verbose':'true',
                'gui_required':'true'
                }.items()
        )
    ])
