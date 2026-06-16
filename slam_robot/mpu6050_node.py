#!/usr/bin/env python3
"""
MPU6050 IMU Node — ROS 2 Jazzy
================================
Reads MPU6050 over I2C and publishes to /imu/data
Accel Z = 9.73 confirmed working ✅
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math

try:
    from mpu6050 import mpu6050
    IMU_AVAILABLE = True
except ImportError:
    IMU_AVAILABLE = False


class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')

        if not IMU_AVAILABLE:
            self.get_logger().error(
                'mpu6050 library not found!')
            self.get_logger().error(
                'Run: pip3 install mpu6050-raspberrypi --break-system-packages')
            return

        # ── Connect to sensor ─────────────────────────────────
        try:
            self.sensor = mpu6050(0x68)
            self.get_logger().info('MPU6050 connected at 0x68 ✅')
        except Exception as e:
            self.get_logger().error(f'MPU6050 failed: {e}')
            self.get_logger().error(
                'Check: sudo i2cdetect -y 1 → should show 68')
            return

        # ── Publisher ─────────────────────────────────────────
        self.pub = self.create_publisher(Imu, '/imu/data', 10)

        # ── Timer — 50 Hz ─────────────────────────────────────
        self.timer = self.create_timer(0.02, self.publish_imu)

        self.get_logger().info('Publishing /imu/data at 50Hz')

    def publish_imu(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro  = self.sensor.get_gyro_data()
        except Exception as e:
            self.get_logger().warn(f'IMU read error: {e}')
            return

        msg = Imu()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Accelerometer m/s²
        msg.linear_acceleration.x = accel['x']
        msg.linear_acceleration.y = accel['y']
        msg.linear_acceleration.z = accel['z']

        # Gyroscope deg/s → rad/s
        msg.angular_velocity.x = math.radians(gyro['x'])
        msg.angular_velocity.y = math.radians(gyro['y'])
        msg.angular_velocity.z = math.radians(gyro['z'])

        # Orientation unknown — set covariance -1
        msg.orientation_covariance[0]          = -1.0
        msg.linear_acceleration_covariance[0]  =  0.01
        msg.angular_velocity_covariance[0]     =  0.01

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPU6050Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
