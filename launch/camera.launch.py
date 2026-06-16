"""
USB Camera Only Launch File
=============================
Use this to test camera alone before full system launch.

Usage:
  ros2 launch slam_robot camera.launch.py
  ros2 launch slam_robot camera.launch.py device_id:=1
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg      = get_package_share_directory('slam_robot')
    rviz_cfg = os.path.join(pkg, 'rviz', 'camera_view.rviz')

    device_arg = DeclareLaunchArgument(
        'device_id', default_value='0',
        description='Camera /dev/videoX number')

    # USB Camera Node
    cam_node = Node(
        package='slam_robot',
        executable='usb_cam_node',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'width':     640,
            'height':    480,
            'fps':       30,
            'frame_id':  'camera_link',
        }],
    )

    # Static TF: base_link → camera_link
    # xyz = how far camera is from robot center
    cam_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf',
        arguments=[
            '0.085',      # x: 8.5cm forward
            '0',          # y: centered
            '0.05',       # z: 5cm above base
            '0', '0', '0',
            'base_link', 'camera_link',
        ],
    )

    # RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_camera',
        output='screen',
        arguments=['-d', rviz_cfg],
    )

    return LaunchDescription([
        device_arg,
        cam_node,
        cam_tf,
        rviz,
    ])
