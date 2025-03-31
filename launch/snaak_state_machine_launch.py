import launch
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    vision_launch = IncludeLaunchDescription(
        package='snaak_vision', 
        launch='snaak_vision_launch.py'
        )
    manipulation_launch = IncludeLaunchDescription(
        package='snaak_manipulation', 
        launch='snaak_manipulation_launch.py')      
    weight_launch = IncludeLaunchDescription(
        package='snaak_weight_read', 
        launch='snaak_manipulation_launch.py')  
    state_machine_node = launch_ros.actions.Node(
            package='snaak_state_machine',
            executable='snaak_state_machine_main',
            name='snaak_state_machine',
        )
    return launch.LaunchDescription([
        vision_launch,
        manipulation_launch,
        weight_launch,
        state_machine_node
    ])