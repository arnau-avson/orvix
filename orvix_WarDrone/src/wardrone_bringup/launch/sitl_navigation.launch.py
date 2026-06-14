"""Launch file for Milestone S2: Driver + Navigation + Safety.

Usage:
    ros2 launch wardrone_bringup sitl_navigation.launch.py
    ros2 launch wardrone_bringup sitl_navigation.launch.py mission_file:=/path/to/mission.yaml
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
        DeclareLaunchArgument('mission_file', default_value=''),

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

        # Waypoint Navigator
        Node(
            package='wardrone_navigation',
            executable='waypoint_navigator_node',
            name='waypoint_navigator',
            parameters=[
                os.path.join(bringup_dir, 'config', 'navigation_params.yaml'),
                {'mission_file': LaunchConfiguration('mission_file')},
            ],
            output='screen',
        ),

        # Safety Monitor
        Node(
            package='wardrone_navigation',
            executable='safety_monitor_node',
            name='safety_monitor',
            parameters=[
                os.path.join(bringup_dir, 'config', 'navigation_params.yaml'),
            ],
            output='screen',
        ),
    ])
