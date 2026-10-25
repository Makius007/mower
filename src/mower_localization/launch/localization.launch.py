import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('mower_localization')


    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf.yaml'),
            {'use_sim_time': True}
        ]
    )

    safety_node = Node(
        package='mower_localization',
        executable='safety_mode.py',
        name='safety_mode',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        ekf_node,
        #safety_node
    ])