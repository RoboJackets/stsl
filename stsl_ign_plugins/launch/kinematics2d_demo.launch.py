import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    test_world_path=os.sep.join([get_package_share_directory('stsl_ign_plugins'),'worlds','test.sdf'])
    ros_ign_launch_path=os.sep.join([get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'])

    ign_args=' '.join(['-r',test_world_path])

    env={
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':os.pathsep.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ])
    }

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ros_ign_launch_path),
            launch_arguments={'ign_args':ign_args}.items()
        )
    ])
