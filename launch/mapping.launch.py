"""
Map Creation Launch File
"""
import os
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            'serial_port':   '/dev/ttyUSB1',
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
            'serial_port':      '/dev/ttyUSB0',
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
            parameters=[params, {'use_sim_time': False}],
        )]
    )

    activate_slam = TimerAction(
        period=15.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 'source /opt/ros/jazzy/setup.bash && ros2 lifecycle set /slam_toolbox configure && sleep 1 && ros2 lifecycle set /slam_toolbox activate'],
                output='screen'
            )
        ]
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

    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    map_saver = Node(
        package='slam_robot',
        executable='map_saver_node',
        name='map_saver_node',
        output='screen'
    )

    coverage_planner = Node(
        package='slam_robot',
        executable='coverage_planner_node',
        name='coverage_planner_node',
        output='screen'
    )

    return LaunchDescription([
        rsp, serial, lidar, imu, slam, activate_slam, rviz, rosbridge, map_saver, coverage_planner
    ])
