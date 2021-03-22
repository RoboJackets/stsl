import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    test_world_path=os.sep.join([get_package_share_directory('traini_gazebo'),'worlds','training_world.sdf'])
    ros_ign_launch_path=os.sep.join([get_package_share_directory('ros_ign_gazebo'),'launch','ign_gazebo.launch.py'])
    resource_path=os.sep.join([get_package_share_directory('traini_gazebo'),'models'])

    return LaunchDescription([
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', os.environ['LD_LIBRARY_PATH']),
        ExecuteProcess(
            cmd=['ign gazebo', '-r', test_world_path],
            output='screen',
            on_exit=Shutdown(),
            shell=True
        ),
        Node(
            name='ros_ign_bridge_cmd_vel',
            package='ros_ign_bridge',
            executable='parameter_bridge',
            output='screen',
            arguments=['/model/Traini/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
            remappings=[
                ('/model/Traini/cmd_vel','cmd_vel')
            ]
        )
    ])
