import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_controller = get_package_share_directory('mower_controller')
    controllers_config = os.path.join(
        pkg_project_controller, 'config', 'project_controllers.yaml'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            controllers_config,
        ],
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--param-file',
            controllers_config,
            '--ros-args',
            '-r',
            '/diff_drive_controller/cmd_vel_unstamped:=/cmd_vel',
        ],
    )

    mowing_alghoritm = Node(
            package='mower_controller',
            executable='mowing_node.py',
            name='mowing_alghoritm_node',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

    grass_detector_node = Node(
            package='mower_controller',
            executable='grass_detector.py',
            name='grass_detector',
            parameters=[{'use_sim_time': True}],
            output='screen'
        )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
            grass_detector_node,
            mowing_alghoritm
        ]
    )