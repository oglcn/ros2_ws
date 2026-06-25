"""
ArUco approach launch file -- drives the robot toward a target marker.

Launches camera, ArUco detector, motor driver, the approach controller,
and the lightweight ArUco test page (for watching the camera/detections
while it drives). No manual control is exposed here on purpose.

Usage:
  ros2 launch delivery_robot_bringup aruco_approach.launch.py target_marker_id:=0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('delivery_robot_bringup')

    camera_config = os.path.join(bringup_dir, 'config', 'camera.yaml')
    motor_config = os.path.join(bringup_dir, 'config', 'motor_pins.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('target_marker_id', default_value='0'),
        DeclareLaunchArgument('web_port', default_value='8081'),

        Node(
            package='pi_camera_driver',
            executable='camera_node',
            name='pi_camera',
            parameters=[camera_config],
            output='screen',
        ),

        Node(
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
            package='motor_driver',
            executable='motor_driver_node',
            name='motor_driver',
            parameters=[motor_config],
            output='screen',
        ),

        Node(
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
            package='robot_web_ui',
            executable='aruco_test_node',
            name='aruco_test_web',
            parameters=[{'port': LaunchConfiguration('web_port')}],
            output='screen',
        ),
    ])
