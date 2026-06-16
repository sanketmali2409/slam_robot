from setuptools import find_packages, setup
from glob import glob

package_name = 'slam_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,               ['package.xml']),
        ('share/' + package_name + '/launch',   glob('launch/*.py')),
        ('share/' + package_name + '/config',   glob('config/*.yaml')),
        ('share/' + package_name + '/urdf',     glob('urdf/*')),
        ('share/' + package_name + '/rviz',     glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@robot.local',
    description='4WD Mecanum SLAM robot with USB Camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'real_odom_node         = slam_robot.real_odom_node:main',
            'cmd_vel_to_serial_node = slam_robot.cmd_vel_to_serial_node:main',
            'serial_bridge_node     = slam_robot.serial_bridge_node:main',
            'coverage_planner_node  = slam_robot.coverage_planner_node:main',
            'mpu6050_node           = slam_robot.mpu6050_node:main',
            'usb_cam_node           = slam_robot.usb_cam_node:main',
        ],
    },
)
