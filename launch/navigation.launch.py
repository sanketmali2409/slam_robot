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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg        = get_package_share_directory('slam_robot')
    nav2_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    urdf_file   = os.path.join(pkg, 'urdf',  'robot.urdf.xml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg, 'maps', 'room_map.yaml'),
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

    # ── Serial Bridge (Odom + CMD_VEL) ───────────────────────
    serial_bridge = Node(
        package='slam_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_smoothed')],
        parameters=[{
            'serial_port':   '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'baud_rate':     115200,
            'wheel_radius':  0.03,
            'wheel_base_y':  0.185,
            'wheel_base_x':  0.0425,
            'encoder_ppr':   360,
            'max_speed':     1000,
        }],
    )

    # ── RPLIDAR ──────────────────────────────────────────────
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
            'serial_baudrate':  115200,
            'frame_id':         'base_laser',
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    # ── Nav2 Bringup ─────────────────────────────────────────
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': nav2_params,
            'use_sim_time': 'False',
        }.items()
    )

    # ── Dashboard Connectivity ───────────────────────────────
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    map_repub = Node(
        package='slam_robot',
        executable='map_republisher',
        name='map_republisher',
        output='screen'
    )

    goal_bridge = Node(
        package='slam_robot',
        executable='goal_pose_bridge',
        name='goal_pose_bridge',
        output='screen'
    )

    # ── IMU ──────────────────────────────────────────────────
    imu_node = Node(
        package='slam_robot',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen'
    )

    # ── RViz ─────────────────────────────────────────────────
    rviz_cfg = os.path.join(pkg, 'rviz', 'nav_view.rviz')
    # fallback to slam_view if nav_view doesn't exist
    if not os.path.exists(rviz_cfg):
        rviz_cfg = os.path.join(pkg, 'rviz', 'slam_view.rviz')
        
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    return LaunchDescription([
        map_arg,
        robot_state_publisher,
        rplidar,
        serial_bridge,
        nav2,
        rosbridge,
        map_repub,
        goal_bridge,
        imu_node,
        rviz
    ])
