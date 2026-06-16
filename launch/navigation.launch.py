"""
Navigation Launch File
=======================
Loads saved map + starts Nav2 stack
Run AFTER map is saved

Usage:
  ros2 launch slam_robot navigation.launch.py
  ros2 launch slam_robot navigation.launch.py \
    map:=/home/rasppi/my_room_map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg        = get_package_share_directory('slam_robot')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz',  'nav2_view.rviz')
    urdf_file   = os.path.join(pkg, 'urdf',  'robot.urdf.xml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/my_room_map.yaml'),
        description='Full path to map yaml file')

    # ── Robot State Publisher ────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ── Real Odometry ────────────────────────────────────────
    real_odom = Node(
        package='slam_robot',
        executable='real_odom_node',
        name='real_odom_node',
        output='screen',
        parameters=[{
            'serial_port':   '/dev/ttyUSB1',
            'baud_rate':     115200,
            'encoder_ppr':   360,
            'wheel_radius':  0.03,
            'wheel_base_y':  0.185,
            'wheel_base_x':  0.0425,
        }],
    )

    # ── RPLIDAR ──────────────────────────────────────────────
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/ttyUSB0',
            'serial_baudrate':  115200,
            'frame_id':         'base_laser',
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    # ── CMD_VEL Serial Bridge ────────────────────────────────
    serial_bridge = Node(
        package='slam_robot',
        executable='cmd_vel_to_serial_node',
        name='cmd_vel_to_serial_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'baud_rate':   115200,
            'max_speed':   200,
        }],
    )

    # ── Nav2 Bringup ─────────────────────────────────────────
    nav2 = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='nav2_bringup',
        output='screen',
        parameters=[
            nav2_params,
            {'map': LaunchConfiguration('map')},
            {'use_sim_time': False},
        ],
    )

    # ── RViz2 ────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        map_arg,
        robot_state_publisher,
        real_odom,
        rplidar,
        serial_bridge,
        nav2,
        rviz,
    ])
