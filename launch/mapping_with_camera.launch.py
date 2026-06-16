"""
Complete Mapping Launch — ALL Sensors Including Camera
=======================================================
Starts:
  1. Robot State Publisher
  2. Serial Bridge  (ESP32 /dev/ttyUSB1)
  3. RPLIDAR        (/dev/ttyUSB0)
  4. MPU6050 IMU    (I2C)
  5. USB Camera     (/dev/video0)
  6. SLAM Toolbox
  7. RViz2 with map + camera view

Run:
  ros2 launch slam_robot mapping_with_camera.launch.py

Save map when done:
  ros2 run nav2_map_server map_saver_cli -f ~/my_room_map
"""

import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg       = get_package_share_directory('slam_robot')
    params    = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    rviz_cfg  = os.path.join(pkg, 'rviz',   'slam_camera_view.rviz')
    urdf_file = os.path.join(pkg, 'urdf',   'robot.urdf.xml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── 1. Robot State Publisher ──────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # ── 2. Serial Bridge — ESP32 ──────────────────────────────
    serial = Node(
        package='slam_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port':   '/dev/ttyUSB1',
            'baud_rate':     115200,
            'wheel_radius':  0.03,
            'wheel_base_y':  0.185,
            'wheel_base_x':  0.0425,
            'encoder_ppr':   360,
            'max_speed':     200,
        }],
    )

    # ── 3. RPLIDAR ────────────────────────────────────────────
    lidar = Node(
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

    # ── 4. MPU6050 IMU ────────────────────────────────────────
    imu = Node(
        package='slam_robot',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
    )

    # ── 5. USB Camera ─────────────────────────────────────────
    camera = Node(
        package='slam_robot',
        executable='usb_cam_node',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'device_id': 0,     # /dev/video0 — change if needed
            'width':     640,
            'height':    480,
            'fps':       30,
            'frame_id':  'camera_link',
        }],
    )

    # Camera TF: base_link → camera_link
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '0.085', '0', '0.05',
            '0', '0', '0',
            'base_link', 'camera_link',
        ],
    )

    # ── 6. SLAM Toolbox — 3s delay ────────────────────────────
    slam = TimerAction(
        period=3.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params],
        )]
    )

    # ── 7. RViz2 — 5s delay ───────────────────────────────────
    rviz = TimerAction(
        period=5.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_cfg],
        )]
    )

    return LaunchDescription([
        rsp, serial, lidar, imu,
        camera, cam_tf,
        slam, rviz,
    ])
