"""
Localization launch file for the delivery robot.

Launches ORB-SLAM3, optional ArUco detector, and robot_localization EKF.
This should be launched alongside the main bringup.launch.py.

Usage:
  ros2 launch delivery_robot_bringup localization.launch.py
  ros2 launch delivery_robot_bringup localization.launch.py use_aruco:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('delivery_robot_bringup')

    ekf_config = os.path.join(bringup_dir, 'config', 'ekf.yaml')
    default_markers = os.path.join(bringup_dir, 'config', 'aruco_markers.yaml')

    orb_slam3_vocab = '/home/pi/third_party/ORB_SLAM3/Vocabulary/ORBvoc.txt'
    orb_slam3_settings = '/home/pi/ros2_ws/src/delivery_robot/orb_slam3_ros/config/orb_slam3_pi5.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('use_aruco', default_value='true',
                              description='Launch ArUco detector'),
        DeclareLaunchArgument('use_ekf', default_value='true',
                              description='Launch robot_localization EKF'),
        DeclareLaunchArgument('use_vslam', default_value='true',
                              description='Launch ORB-SLAM3 visual odometry'),
        DeclareLaunchArgument('marker_map_file', default_value=default_markers,
                              description='Path to ArUco marker map YAML file'),

        # ORB-SLAM3 visual odometry
        Node(
            condition=IfCondition(LaunchConfiguration('use_vslam')),
            package='orb_slam3_ros',
            executable='orb_slam3_node',
            name='orb_slam3',
            parameters=[{
                'vocabulary_file': orb_slam3_vocab,
                'settings_file': orb_slam3_settings,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'camera_frame': 'camera_link',
            }],
            output='screen',
        ),

        # ArUco marker detector for localization (publishes /aruco/pose in map frame)
        Node(
            condition=IfCondition(LaunchConfiguration('use_aruco')),
            package='aruco_detector',
            executable='aruco_detector_node',
            name='aruco_localizer',
            parameters=[{
                'marker_size': 0.17,
                'dictionary': 'DICT_4X4_50',
                'marker_map_file': LaunchConfiguration('marker_map_file'),
                'detection_rate_hz': 10.0,
            }],
            output='screen',
        ),

        # robot_localization EKF
        Node(
            condition=IfCondition(LaunchConfiguration('use_ekf')),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config],
            output='screen',
        ),
    ])
