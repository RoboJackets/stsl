import os
import launch
import launch.actions
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    test_world_path=os.sep.join([get_package_share_directory('ign_kinematics_plugin'),'worlds','test.sdf'])

    env={
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':os.pathsep.join([
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ])
    }

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ign gazebo', '-r', '-v 4', test_world_path],
            output='screen',
            additional_env=env,
            shell=True
        )
    ])
