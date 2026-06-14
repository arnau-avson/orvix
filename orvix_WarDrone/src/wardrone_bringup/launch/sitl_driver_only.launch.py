"""Launch file for Milestone S1: MAVSDK bridge only.

Usage:
    ros2 launch wardrone_bringup sitl_driver_only.launch.py

Verify with:
    ros2 topic echo /wardrone/telemetry
    ros2 service call /wardrone/arm wardrone_interfaces/srv/Arm "{arm: true}"
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
        DeclareLaunchArgument(
            'connection_url',
            default_value='udp://:14540',
            description='MAVSDK connection URL'
        ),

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
    ])
