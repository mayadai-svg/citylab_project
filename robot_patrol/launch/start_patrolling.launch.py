import launch
import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rviz_config_dir = '/home/user/ros2_ws/src/citylab_project/robot_patrol/src/robot_patrol_config.rviz'
    
    # Nodes
    patrol_node = launch_ros.actions.Node(
        package='robot_patrol',
        executable='patrol',
        arguments=[],
        output='screen',
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        # arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
       patrol_node,
       rviz_node
    ])