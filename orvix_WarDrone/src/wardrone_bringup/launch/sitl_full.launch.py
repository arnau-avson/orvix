"""Launch file for Milestone S5: Full simulation stack.

Launches all nodes for a complete autonomous mission in PX4 SITL + Gazebo.

Usage:
    ros2 launch wardrone_bringup sitl_full.launch.py
    ros2 launch wardrone_bringup sitl_full.launch.py mission_file:=/path/to/mission.yaml
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
        # --- Arguments ---
        DeclareLaunchArgument('connection_url', default_value='udp://:14540'),
        DeclareLaunchArgument('mission_file', default_value=''),
        DeclareLaunchArgument('mission_type', default_value='navigate_and_track'),
        DeclareLaunchArgument('enable_sim_ground_truth', default_value='true'),
        DeclareLaunchArgument('enable_vio_eval', default_value='true'),

        # --- MAVSDK Bridge ---
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

        # --- Navigation ---
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

        Node(
            package='wardrone_navigation',
            executable='safety_monitor_node',
            name='safety_monitor',
            parameters=[
                os.path.join(bringup_dir, 'config', 'navigation_params.yaml'),
            ],
            output='screen',
        ),

        # --- Vision ---
        Node(
            package='wardrone_vision',
            executable='camera_node',
            name='camera',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vision_params.yaml'),
            ],
            output='screen',
        ),

        Node(
            package='wardrone_vision',
            executable='detector_node',
            name='detector',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vision_params.yaml'),
            ],
            output='screen',
        ),

        Node(
            package='wardrone_vision',
            executable='tracker_node',
            name='tracker',
            parameters=[
                os.path.join(bringup_dir, 'config', 'vision_params.yaml'),
            ],
            output='screen',
        ),

        # --- Obstacle Detection & Avoidance ---
        Node(
            package='wardrone_navigation',
            executable='obstacle_detector_node',
            name='obstacle_detector',
            parameters=[
                os.path.join(bringup_dir, 'config', 'obstacle_params.yaml'),
            ],
            output='screen',
        ),

        # --- Obstacle Merger (fuses vision + range) ---
        Node(
            package='wardrone_navigation',
            executable='obstacle_merger_node',
            name='obstacle_merger',
            parameters=[
                os.path.join(bringup_dir, 'config', 'navigation_params.yaml'),
            ],
            output='screen',
        ),

        Node(
            package='wardrone_navigation',
            executable='obstacle_avoidance_node',
            name='obstacle_avoidance',
            parameters=[
                os.path.join(bringup_dir, 'config', 'obstacle_params.yaml'),
            ],
            output='screen',
        ),

        # --- VIO ---
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

        # --- Wind Estimator (F9) ---
        Node(
            package='wardrone_navigation',
            executable='wind_estimator_node',
            name='wind_estimator',
            parameters=[
                os.path.join(bringup_dir, 'config', 'navigation_params.yaml'),
            ],
            output='screen',
        ),

        # --- Flight Logger (F1) ---
        Node(
            package='wardrone_navigation',
            executable='flight_logger_node',
            name='flight_logger',
            parameters=[
                os.path.join(bringup_dir, 'config', 'navigation_params.yaml'),
            ],
            output='screen',
        ),

        # --- Mission Controller ---
        Node(
            package='wardrone_mission',
            executable='mission_controller_node',
            name='mission_controller',
            parameters=[
                os.path.join(bringup_dir, 'config', 'mission_params.yaml'),
                {
                    'mission_file': LaunchConfiguration('mission_file'),
                    'mission_type': LaunchConfiguration('mission_type'),
                },
            ],
            output='screen',
        ),
    ])
