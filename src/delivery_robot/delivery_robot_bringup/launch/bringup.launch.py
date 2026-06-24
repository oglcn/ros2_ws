import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('delivery_robot_bringup')

    camera_config = os.path.join(bringup_dir, 'config', 'camera.yaml')
    motor_config = os.path.join(bringup_dir, 'config', 'motor_pins.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('web_port', default_value='8080'),

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
            parameters=[{'port': LaunchConfiguration('web_port')}],
            output='screen',
        ),

        # Static TF: base_link -> camera_link
        # Camera mounted forward-facing, ~0.10m above base center, ~0.05m forward
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
    ])
