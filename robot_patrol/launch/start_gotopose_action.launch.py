import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():    
    # Nodes
    action_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='go_to_pose_action',
        name= 'go_to_pose_action',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
       action_node
    ])