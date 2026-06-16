#!/usr/bin/env python3
"""
Real Odometry Node — Mecanum Wheel
====================================
YOUR ROBOT MEASUREMENTS:
  Wheel diameter      : 60mm  → radius = 0.03m
  Left-Right distance : 37cm  → wheel_base_y = 0.185m
  Front-Back distance : measure and update wheel_base_x
  Encoder PPR         : 360
  Meters per pulse    : 0.000524m (0.524mm)
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import math


class RealOdomNode(Node):
    def __init__(self):
        super().__init__('real_odom_node')

        # ═══════════════════════════════════════════════════
        #  YOUR EXACT ROBOT PARAMETERS
        # ═══════════════════════════════════════════════════
        self.declare_parameter('wheel_radius',  0.03)   # 60mm dia / 2 = 0.03m ✅
        self.declare_parameter('wheel_base_y',  0.185)  # 37cm / 2 = 0.185m    ✅
        self.declare_parameter('wheel_base_x',  0.185)  # front-back half dist
                                                         # ← measure and update!
        self.declare_parameter('encoder_ppr',   360)    # 360 PPR               ✅
        self.declare_parameter('serial_port',   '/dev/ttyUSB1')
        self.declare_parameter('baud_rate',     115200)

        self.R   = self.get_parameter('wheel_radius').value
        self.Ly  = self.get_parameter('wheel_base_y').value
        self.Lx  = self.get_parameter('wheel_base_x').value
        self.PPR = self.get_parameter('encoder_ppr').value
        port     = self.get_parameter('serial_port').value
        baud     = self.get_parameter('baud_rate').value

        # Meters per encoder pulse = circumference / PPR
        # = (2 × π × 0.03) / 360 = 0.000524m
        self.mpp = (2.0 * math.pi * self.R) / self.PPR

        self.get_logger().info('=' * 48)
        self.get_logger().info('  Mecanum Odometry — Your Robot Values')
        self.get_logger().info('=' * 48)
        self.get_logger().info(f'  Wheel radius   : {self.R*1000:.0f} mm  (60mm dia)')
        self.get_logger().info(f'  Wheel base Y   : {self.Ly*100:.1f} cm  (half of 37cm)')
        self.get_logger().info(f'  Wheel base X   : {self.Lx*100:.1f} cm  (measure front-back!)')
        self.get_logger().info(f'  Encoder PPR    : {self.PPR}')
        self.get_logger().info(f'  Meters / pulse : {self.mpp*1000:.4f} mm')
        self.get_logger().info(f'  Pulses / meter : {1.0/self.mpp:.1f}')
        self.get_logger().info('=' * 48)

        # ── Serial ────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'  Serial port    : {port}')
        except Exception as e:
            self.get_logger().error(f'  Serial FAILED  : {e}')
            self.ser = None

        # ── Robot pose ─────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Previous encoder ticks
        self.prev_FL     = 0
        self.prev_FR     = 0
        self.prev_BL     = 0
        self.prev_BR     = 0
        self.first_read  = True

        # ── ROS publishers ─────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br    = tf2_ros.TransformBroadcaster(self)

        # ── 20Hz timer ─────────────────────────────────────
        self.timer = self.create_timer(0.05, self.read_serial)
        self.get_logger().info('  Publishing /odom + TF odom→base_link ✅')

    # ════════════════════════════════════════════════════════
    #  READ SERIAL FROM ESP32
    # ════════════════════════════════════════════════════════
    def read_serial(self):
        if self.ser is None:
            return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode(
                    'utf-8', errors='ignore').strip()

                if line.startswith('ENC'):
                    parts = line.split()
                    if len(parts) == 5:
                        fl = int(parts[1])
                        fr = int(parts[2])
                        bl = int(parts[3])
                        br = int(parts[4])

                        # First reading: set baseline only
                        if self.first_read:
                            self.prev_FL = fl
                            self.prev_FR = fr
                            self.prev_BL = bl
                            self.prev_BR = br
                            self.first_read = False
                            return

                        self.compute_odom(fl, fr, bl, br)

        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    # ════════════════════════════════════════════════════════
    #  MECANUM FORWARD KINEMATICS
    #
    #  Robot top view:
    #
    #       |←──── 0.37m ────→|
    #   FL \────────────────/ FR   ─┐
    #      \\                //     |
    #       \\              //      | measure
    #      //                \\     | this!
    #   BL /────────────────\ BR   ─┘
    #
    #  Equations:
    #    vx    =  (FL + FR + BL + BR) / 4
    #    vy    = (-FL + FR + BL - BR) / 4
    #    omega = (-FL + FR - BL + BR) / (4*(Lx+Ly))
    # ════════════════════════════════════════════════════════
    def compute_odom(self, fl, fr, bl, br):

        # Delta ticks since last update
        d_FL = fl - self.prev_FL
        d_FR = fr - self.prev_FR
        d_BL = bl - self.prev_BL
        d_BR = br - self.prev_BR

        self.prev_FL = fl
        self.prev_FR = fr
        self.prev_BL = bl
        self.prev_BR = br

        # Ticks → meters using 0.000524m/pulse
        d_fl = d_FL * self.mpp
        d_fr = d_FR * self.mpp
        d_bl = d_BL * self.mpp
        d_br = d_BR * self.mpp

        # Mecanum kinematics → body velocities
        vx    = ( d_fl + d_fr + d_bl + d_br) / 4.0
        vy    = (-d_fl + d_fr + d_bl - d_br) / 4.0
        omega = (-d_fl + d_fr - d_bl + d_br) / (4.0 * (self.Lx + self.Ly))

        # Midpoint integration (more accurate)
        half_theta = self.theta + omega / 2.0
        self.x    += vx * math.cos(half_theta) - vy * math.sin(half_theta)
        self.y    += vx * math.sin(half_theta) + vy * math.cos(half_theta)
        self.theta += omega

        # Keep angle within -π to +π
        self.theta = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta))

        self.publish_odom()

    # ════════════════════════════════════════════════════════
    #  PUBLISH /odom + TF
    # ════════════════════════════════════════════════════════
    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        qz  = math.sin(self.theta / 2.0)
        qw  = math.cos(self.theta / 2.0)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp            = now
        t.header.frame_id         = 'odom'
        t.child_frame_id          = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw
        self.tf_br.sendTransform(t)

        # /odom topic
        odom = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariance — tuned for 360PPR encoder accuracy
        odom.pose.covariance[0]  = 0.001  # x
        odom.pose.covariance[7]  = 0.001  # y
        odom.pose.covariance[35] = 0.001  # yaw

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = RealOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
