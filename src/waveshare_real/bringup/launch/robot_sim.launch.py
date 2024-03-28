import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    package_name='waveshare_real'

    declared_arguments = []
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("waveshare_real"),
                    "rover.urdf.xacro",
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    ignore_time = {"ignore_timestamp": False}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("waveshare_real"),
            "config",
            "my_controllers.yaml",
        ]
    )
    robot_localization_file = PathJoinSubstitution(
        [
            FindPackageShare("waveshare_real"),
            "config",
            "ekf.yaml",
        ]
    )

    imu_filter_config = PathJoinSubstitution(
        [
            FindPackageShare("waveshare_real"),
            "config",
            "madgwick.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, ignore_time],
        remappings=[
            ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["waveshare_real_controller", "--controller-manager", "/controller_manager"],
    )


        # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[robot_localization_file],
    )

    imu_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_node",
        output="screen",
        parameters=[imu_filter_config, {"use_mag": False}, {"publish_tf": False}],
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'lidar_link',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]        
    )


    nodes = [
        control_node,
        robot_state_pub_node,
        joint_broad_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        robot_localization,
        imu_filter,
        lidar_node,
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments+nodes)