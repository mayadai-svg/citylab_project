import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    # Nodes
    test_service_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='test_service',
        name= 'test_service',
        arguments=[],
        # emulate_tty=True,
        output='screen',
    )

    return launch.LaunchDescription([
       test_service_node
    ])