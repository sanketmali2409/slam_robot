"""
Nav2 2D Kinematic Simulation Launch File
This runs Nav2 with a saved map and a fake odometry node instead of real hardware.
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('slam_robot')
    
    # Arguments
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg, 'maps', 'my_map.yaml'))
    nav2_params_file = LaunchConfiguration('params_file', default=os.path.join(pkg, 'config', 'nav2_params.yaml'))

    # URDF
    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf.xml')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}]
    )

    # Fake Robot Simulation Node
    fake_robot = Node(
        package='slam_robot',
        executable='fake_odom_node',
        name='fake_odom_node',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_smoothed')]
    )

    # Nav2 Bringup (Path Planning, Costmaps, Lifecycle Management)
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': nav2_params_file,
            'use_sim_time': 'False',
            'use_localization': 'True'
        }.items()
    )

    # ROSBridge for Web Dashboard connection
    rosbridge = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': 9090}]
    )

    # Static Transform map -> odom (bypasses the need for AMCL and initial pose in simulation)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom']
    )

    # Map Republisher to fix ros2djs transient_local issues
    map_repub = Node(
        package='slam_robot',
        executable='map_republisher',
        name='map_republisher',
        output='screen'
    )

    # Goal Pose Bridge to link dashboard's /goal_pose topic to Nav2's navigate_to_pose action
    goal_bridge = Node(
        package='slam_robot',
        executable='goal_pose_bridge',
        name='goal_pose_bridge',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=map_yaml_file, description='Full path to map yaml file to load'),
        rsp,
        fake_robot,
        static_tf,
        nav2_bringup_launch,
        rosbridge,
        map_repub,
        goal_bridge
    ])
