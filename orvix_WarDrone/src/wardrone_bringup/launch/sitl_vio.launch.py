"""Launch file for Milestone S3: Driver + VIO.

Usage:
    ros2 launch wardrone_bringup sitl_vio.launch.py
    ros2 launch wardrone_bringup sitl_vio.launch.py enable_sim_ground_truth:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('wardrone_bringup')

    return LaunchDescription([
        DeclareLaunchArgument('connection_url', default_value='udp://:14540'),
        DeclareLaunchArgument('enable_sim_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_vio_eval', default_value='true'),

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

        # VIO Bridge
        Node(
            package='wardrone_vio',
            executable='vio_bridge_node',
            name='vio_bridge',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vio_params.yaml'),
                {'enable_sim_ground_truth': LaunchConfiguration('enable_sim_ground_truth')},
            ],
            output='screen',
        ),

        # VIO Evaluator (sim only)
        Node(
            package='wardrone_vio',
            executable='vio_evaluator_node',
            name='vio_evaluator',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vio_params.yaml'),
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_vio_eval')),
        ),
    ])
