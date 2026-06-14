"""Launch file for Milestone S4: Driver + Vision pipeline.

Usage:
    ros2 launch wardrone_bringup sitl_vision.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('wardrone_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('connection_url', default_value='udp://:14540'),

        # MAVSDK Bridge
        Node(
            package='wardrone_driver',
            executable='mavsdk_bridge_node',
            name='mavsdk_bridge',
            parameters=[
                os.path.join(bringup_dir, 'config', 'driver_params.yaml'),
                {'connection_url': LaunchConfiguration('connection_url')},
            ],
            output='screen',
        ),

        # Camera
        Node(
            package='wardrone_vision',
            executable='camera_node',
            name='camera',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vision_params.yaml'),
            ],
            output='screen',
        ),

        # Detector
        Node(
            package='wardrone_vision',
            executable='detector_node',
            name='detector',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vision_params.yaml'),
            ],
            output='screen',
        ),

        # Tracker
        Node(
            package='wardrone_vision',
            executable='tracker_node',
            name='tracker',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vision_params.yaml'),
            ],
            output='screen',
        ),
    ])
