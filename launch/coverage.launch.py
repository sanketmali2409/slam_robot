"""
Coverage Planning Launch File
==============================
Launches:
  1. coverage_planner_node  — path planning + Nav2 waypoint sender
  2. RViz2 with coverage visualization config

Usage:
  ros2 launch slam_robot coverage.launch.py
  ros2 launch slam_robot coverage.launch.py auto_start:=true
  ros2 launch slam_robot coverage.launch.py row_direction:=y
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('slam_robot')

    # ── Launch Arguments ─────────────────────────────────────
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='Auto start coverage when map is received')

    row_direction_arg = DeclareLaunchArgument(
        'row_direction', default_value='x',
        description='Sweep axis: x = horizontal rows, y = vertical rows')

    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius', default_value='0.20',
        description='Robot radius in meters (sets row spacing)')

    overlap_arg = DeclareLaunchArgument(
        'coverage_overlap', default_value='0.10',
        description='Row overlap in meters')

    # ── Coverage Planner Node ────────────────────────────────
    coverage_node = Node(
        package='slam_robot',
        executable='coverage_planner_node',
        name='coverage_planner_node',
        output='screen',
        parameters=[{
            'robot_radius':     LaunchConfiguration('robot_radius'),
            'coverage_overlap': LaunchConfiguration('coverage_overlap'),
            'row_direction':    LaunchConfiguration('row_direction'),
            'auto_start':       LaunchConfiguration('auto_start'),
            'free_threshold':   50,
            'goal_timeout':     30.0,
            'linear_speed':     0.2,
        }],
    )

    # ── RViz2 ────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_coverage',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'coverage_view.rviz')],
    )

    return LaunchDescription([
        auto_start_arg,
        row_direction_arg,
        robot_radius_arg,
        overlap_arg,
        coverage_node,
        rviz_node,
    ])
