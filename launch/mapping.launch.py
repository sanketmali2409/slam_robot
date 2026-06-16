"""
Map Creation Launch File
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg       = get_package_share_directory('slam_robot')
    params    = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    rviz_cfg  = os.path.join(pkg, 'rviz',   'slam_view.rviz')
    urdf_file = os.path.join(pkg, 'urdf',   'robot.urdf.xml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

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

    serial = Node(
        package='slam_robot',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{
            'serial_port':   '/dev/ttyUSB0',
            'baud_rate':     115200,
            'wheel_radius':  0.03,
            'wheel_base_y':  0.185,
            'wheel_base_x':  0.0425,
            'encoder_ppr':   360,
            'max_speed':     200,
        }],
    )

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen',
        parameters=[{
            'serial_port':      '/dev/ttyUSB1',
            'serial_baudrate':  115200,
            'frame_id':         'base_laser',
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    imu = Node(
        package='slam_robot',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
    )

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
        rsp, serial, lidar, imu, slam, rviz,
    ])
