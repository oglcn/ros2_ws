import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('delivery_robot_bringup')

    camera_config = os.path.join(bringup_dir, 'config', 'camera.yaml')
    motor_config = os.path.join(bringup_dir, 'config', 'motor_pins.yaml')
    imu_config = os.path.join(bringup_dir, 'config', 'imu.yaml')
    delivery_config = os.path.join(bringup_dir, 'config', 'delivery.yaml')
    menu_config = os.path.join(bringup_dir, 'config', 'menu.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('web_port', default_value='8080'),
        DeclareLaunchArgument('use_approach', default_value='true',
                              description='Launch ArUco detector and approach node'),
        DeclareLaunchArgument('use_imu', default_value='true',
                              description='Launch MPU6050 IMU driver'),
        DeclareLaunchArgument('target_marker_id', default_value='0'),
        DeclareLaunchArgument('use_delivery', default_value='true',
                              description='Launch delivery mission manager'),

        Node(
            package='pi_camera_driver',
            executable='camera_node',
            name='pi_camera',
            parameters=[camera_config],
            output='screen',
        ),

        Node(
            package='motor_driver',
            executable='motor_driver_node',
            name='motor_driver',
            parameters=[motor_config],
            output='screen',
        ),

        Node(
            package='robot_web_ui',
            executable='web_ui_node',
            name='web_ui',
            parameters=[{
                'port': LaunchConfiguration('web_port'),
                'menu_file': menu_config,
            }],
            output='screen',
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            arguments=[
                '--x', '0.05',
                '--y', '0.0',
                '--z', '0.10',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            condition=IfCondition(LaunchConfiguration('use_imu')),
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.05',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_imu')),
            package='imu_driver',
            executable='imu_driver_node',
            name='imu_driver',
            parameters=[imu_config],
            output='screen',
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_approach')),
            package='aruco_detector',
            executable='aruco_detector_node',
            name='aruco_detector',
            parameters=[{
                'marker_size': 0.15,
                'dictionary': 'DICT_4X4_50',
                'marker_map_file': '',
                'detection_rate_hz': 10.0,
            }],
            output='screen',
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_approach')),
            package='aruco_detector',
            executable='aruco_approach_node',
            name='aruco_approach',
            parameters=[{
                'target_marker_id': ParameterValue(
                    LaunchConfiguration('target_marker_id'), value_type=int),
            }],
            output='screen',
        ),

        Node(
            condition=IfCondition(LaunchConfiguration('use_delivery')),
            package='delivery_mission',
            executable='mission_manager_node',
            name='mission_manager',
            parameters=[delivery_config],
            output='screen',
        ),
    ])
