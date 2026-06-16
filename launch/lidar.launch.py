from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'serial_baudrate': 115200,
                'frame_id': 'base_laser',
                'angle_compensate': True,
                'scan_mode': 'Standard',
            }],
        ),
    ])
