#!/usr/bin/env python3
"""
Fake Kinematic Simulator Node
Listens to /cmd_vel, integrates the velocities, and publishes /odom and TF
to simulate the robot without physical hardware.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
import time

class FakeOdomNode(Node):
    def __init__(self):
        super().__init__('fake_odom_node')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br = tf2_ros.TransformBroadcaster(self)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_time = self.get_clock().now()

        # Update loop at 50Hz
        self.create_timer(0.02, self.update_kinematics)
        self.get_logger().info('Fake Odom Node Started! Listening to /cmd_vel')

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def update_kinematics(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Midpoint integration for better accuracy
        half_theta = self.theta + (self.wz * dt) / 2.0
        
        self.x += (self.vx * math.cos(half_theta) - self.vy * math.sin(half_theta)) * dt
        self.y += (self.vx * math.sin(half_theta) + self.vy * math.cos(half_theta)) * dt
        self.theta += self.wz * dt
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        self.publish_odom(now)

    def publish_odom(self, now):
        now_msg = now.to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Transform
        t = TransformStamped()
        t.header.stamp = now_msg
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_br.sendTransform(t)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now_msg
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        
        # Add small covariance
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[35] = 0.001

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = FakeOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
