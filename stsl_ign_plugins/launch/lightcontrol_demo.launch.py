import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    test_world_path=os.sep.join([get_package_share_directory('stsl_ign_plugins'),'worlds','lightcontrol_demo.sdf'])
    ros_ign_launch_path=os.sep.join([get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'])

    ign_args=' '.join(['-r','-v 4','--force-version 5.0.0~pre1',test_world_path])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_ign_launch_path),
            launch_arguments={'ign_args':ign_args}.items()
        )
    ])
