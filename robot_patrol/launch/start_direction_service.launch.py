import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    # Nodes
    service_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='direction_service',
        name= 'direction_service',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
       service_node
    ])