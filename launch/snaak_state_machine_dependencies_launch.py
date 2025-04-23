import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    vision_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('snaak_vision'),
                    'launch',
                    'snaak_vision_launch.py'
                )
            ])
        )
    manipulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('snaak_manipulation'),
                    'launch',
                    'snaak_manipulation_launch.py'
                )
            ])
        )   
    weight_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('snaak_weight_read'),
                    'launch',
                    'snaak_weight_read_launch.py'
                )
            ])
        )   
    # state_machine_node = launch_ros.actions.Node(
    #         package='snaak_state_machine',
    #         executable='snaak_state_machine_main',
    #         name='snaak_state_machine',
    #     )
    return launch.LaunchDescription([
        vision_launch,
        manipulation_launch,
        weight_launch,
        #state_machine_node
    ])