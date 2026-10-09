import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, AppendEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    project_description_dir=get_package_share_directory("mower_description")

    ros_share_dir = os.path.dirname(project_description_dir)


    model_arg=DeclareLaunchArgument(name="model", default_value=os.path.join(
        project_description_dir,"urdf", "mower.urdf.xacro"
    ),
        description="Absolute path to robot urdf file"
    )

    world_path = PathJoinSubstitution([
        FindPackageShare("mower_description"),
        "worlds",
        "lawn.sdf"
    ])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": ["-r -v 4 ", world_path]
        }.items()
    )

    robot_description = ParameterValue(
    Command(["xacro ", LaunchConfiguration("model"), " robot_name:=", "robot"]),
    value_type=str
)
    
    robot_state_publisher_node=Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description,
                         "use_sim_time": True}]
        )
    gz_spawn_entity_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_entity_robot",
        arguments=[
            "-topic", "/robot_description",
            "-name", "robot",
            "-x", "9.6",
            "-y", "-6.0",
            "-z", "0.15",
            "-Y", "-1.5708"  
        ],
        output="screen",
    )

    '''arguments=[                              dobry spawn robota
                "-topic", "/robot_description",
                "-name", "robot",
                "-x", "9.6",
                "-y", "-5.0",
                "-z", "0.15",
                "-Y", "-1.5708"  
            ],'''

    '''arguments=[
                "-topic", "/robot_description",
                "-name", "robot",
                "-x", "-9.6",
                "-y", "-5.0",
                "-z", "0.15",
                "-Y", "1.5708"  
            ],'''

    gz_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )


    gz_camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_camera_bridge",
        arguments=[
            "/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/imu/data_raw@sensor_msgs/msg/Imu[ignition.msgs.IMU",
        ],
        
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    gz_laser_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_laser_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )

    gz_joint_states_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_joint_states_bridge",
        arguments=[
            "/world/partitioned_world/model/robot/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"
        ],
        remappings=[
            ("/world/partitioned_world/model/robot/joint_state", "/joint_states")
        ],
        parameters=[{"use_sim_time": True}],
        output="screen"
    )
    '''apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect', '/camera/image'),
            ('camera_info', '/camera/camera_info'), 
        ],
        parameters=[
            os.path.join(project_description_dir, 'config', 'apriltag.yaml'),
            {
                'approximate_sync': True,
                'use_sim_time': True,
            }  
        ],
        output='screen',
    )'''

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        output="screen",
        arguments=["-d", os.path.join(project_description_dir, "rviz", "urdf.rviz")],
    )


    return LaunchDescription([
        SetEnvironmentVariable('XACRO_VERBOSE', '1'),
        
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=ros_share_dir
        ),
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=ros_share_dir
        ),

        model_arg,
        gazebo,
        gz_clock_bridge,
        robot_state_publisher_node,
        gz_spawn_entity_robot,
        gz_joint_states_bridge,
        gz_camera_bridge,
        gz_laser_bridge,
        rviz_node,
    ])
