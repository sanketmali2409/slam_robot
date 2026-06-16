import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('slam_robot')
    params_file = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz',   'slam_view.rviz')
    urdf_file   = os.path.join(pkg, 'urdf',   'robot.urdf.xml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

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

    real_odom = Node(
        package='slam_robot',
        executable='real_odom_node',
        name='real_odom_node',
        output='screen',
        parameters=[{
            'serial_port':   '/dev/ttyUSB1',
            'baud_rate':     115200,
            'encoder_ppr':   360,    # ✅ your motor
            'wheel_radius':  0.03,   # ✅ 60mm / 2
            'wheel_base_y':  0.185,  # ✅ 37cm / 2
            'wheel_base_x':  0.0425,  # ← update after measuring
                                     #   front-to-back distance
        }],
    )

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

    slam = TimerAction(
        period=3.0,
        actions=[Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[params_file],
        )]
    )

    rviz = TimerAction(
        period=5.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )]
    )

    return LaunchDescription([
        robot_state_publisher,
        real_odom,
        rplidar,
        serial_bridge,
        slam,
        rviz,
    ])
