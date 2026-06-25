"""
ArUco-only test launch file for the delivery robot.

Launches just the camera and the ArUco detector (no motor driver, no
ORB-SLAM3/EKF) plus a minimal web page for picking a target marker ID and
watching whether the camera currently sees it. Intended for bench-testing
marker detection without driving the robot.

Usage:
  ros2 launch delivery_robot_bringup aruco_test.launch.py
  ros2 launch delivery_robot_bringup aruco_test.launch.py web_port:=8081
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('delivery_robot_bringup')

    camera_config = os.path.join(bringup_dir, 'config', 'camera.yaml')
    aruco_markers_config = os.path.join(bringup_dir, 'config', 'aruco_markers.yaml')

    return LaunchDescription([
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
                'marker_map_file': aruco_markers_config,
                'detection_rate_hz': 10.0,
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
