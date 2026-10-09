import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_project_controller = get_package_share_directory('mower_controller')
    controllers_config = os.path.join(
        pkg_project_controller, 'config', 'project_controllers.yaml'
    )

    apriltag_params = os.path.join(
    get_package_share_directory('mower_description'), 'config', 'apriltag.yaml')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file',
            controllers_config,
        ],
    )

    wheel_velocity_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'wheel_velocity_controller',
        '--param-file',
        controllers_config,
    ],
)

    drive_node = Node(
        package='mower_controller',
        executable='drive_node.py',
        name='drive_node',
        parameters=[{'use_sim_time': True}],
        output='screen'
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

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        parameters=[apriltag_params, {'use_sim_time': True}],
    )

    tag_feedback_node = Node(
        package='mower_controller',
        executable='tag_feedback_node.py',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(
        [
            joint_state_broadcaster_spawner,
            wheel_velocity_spawner,
            drive_node,
            #grass_detector_node,
            #mowing_alghoritm,
            apriltag_node,
            tag_feedback_node
        ]
    )