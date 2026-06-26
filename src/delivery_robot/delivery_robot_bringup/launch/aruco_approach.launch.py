"""
ArUco approach launch -- composable with bringup.launch.py.

Starts only the ArUco detector (detection-only, no marker map) and the
approach controller.  Camera, motor driver, web UI, and TF must already
be running via bringup.launch.py.

Usage:
  # Terminal 1 -- base robot
  ~/start_robot.sh

  # Terminal 2 -- add approach capability
  ros2 launch delivery_robot_bringup aruco_approach.launch.py
  ros2 launch delivery_robot_bringup aruco_approach.launch.py target_marker_id:=2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('target_marker_id', default_value='0'),

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
            package='aruco_detector',
            executable='aruco_approach_node',
            name='aruco_approach',
            parameters=[{
                'target_marker_id': ParameterValue(
                    LaunchConfiguration('target_marker_id'), value_type=int),
            }],
            output='screen',
        ),
    ])
