#!/usr/bin/env python3
"""
Serial Bridge Node — Complete Rewrite
Handles all ESP32 output formats automatically
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import serial
import math
import time
import threading


class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('serial_port',  '/dev/ttyUSB1')
        self.declare_parameter('baud_rate',    115200)
        self.declare_parameter('max_speed',    200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_base_y', 0.185)
        self.declare_parameter('wheel_base_x', 0.0425)
        self.declare_parameter('encoder_ppr',  360)

        port         = self.get_parameter('serial_port').value
        baud         = self.get_parameter('baud_rate').value
        self.max_spd = self.get_parameter('max_speed').value
        self.R       = self.get_parameter('wheel_radius').value
        self.Ly      = self.get_parameter('wheel_base_y').value
        self.Lx      = self.get_parameter('wheel_base_x').value
        self.PPR     = self.get_parameter('encoder_ppr').value
        self.mpp     = (2.0 * math.pi * self.R) / self.PPR

        # ── Odometry state ─────────────────────────────────────
        self.x          = 0.0
        self.y          = 0.0
        self.theta      = 0.0
        self.prev_FL    = None
        self.prev_FR    = None
        self.prev_BL    = None
        self.prev_BR    = None
        self.odom_count = 0
        self.enc_count  = 0
        self.last_odom_time = self.get_clock().now()

        # ── ROS ────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_br    = tf2_ros.TransformBroadcaster(self)
        self.cmd_sub  = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ── Serial ─────────────────────────────────────────────
        self.ser  = None
        self.lock = threading.Lock()
        self.connect_serial(port, baud)

        # ── Timers ─────────────────────────────────────────────
        self.create_timer(0.02,  self.read_serial)     # 50Hz
        self.create_timer(0.10,  self.request_enc)     # 10Hz
        self.create_timer(5.0,   self.print_status)    # 5s

        self.get_logger().info('Serial Bridge Node started!')

    # ── Connect Serial ─────────────────────────────────────────
    def connect_serial(self, port, baud):
        try:
            self.ser = serial.Serial()
            self.ser.port = port
            self.ser.baudrate = baud
            self.ser.timeout = 0.05
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            self.ser.open()
            
            time.sleep(2.0)
            self.ser.reset_input_buffer()
            self.get_logger().info(f'Connected: {port} @ {baud}')

            # Ping test with retries and flushing
            for _ in range(5):
                self.ser.write(b'P\n')
                self.ser.flush()
                time.sleep(0.3)
                resp = self.ser.read_all().decode('utf-8', errors='ignore')
                if 'PONG' in resp or 'ENC' in resp:
                    self.get_logger().info('ESP32 Connection Verified ✅')
                    return
                self.ser.reset_input_buffer()

            self.get_logger().warn(f'No valid response during init. Last got: {repr(resp[:80])}')

        except Exception as e:
            self.get_logger().error(f'Serial failed: {e}')
            self.ser = None

    # ── Request Encoders ───────────────────────────────────────
    def request_enc(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            with self.lock:
                self.ser.write(b'G\n')
        except Exception:
            pass

    # ── CMD_VEL Callback ───────────────────────────────────────
    def cmd_vel_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return
        vx = max(-255, min(255, int(msg.linear.x  * self.max_spd)))
        vy = max(-255, min(255, int(msg.linear.y  * self.max_spd)))
        wz = max(-255, min(255, int(msg.angular.z * self.max_spd)))
        
        # Log the command so we can see what's being sent
        self.get_logger().info(f'CMD_VEL: raw_x={msg.linear.x:.3f}, vx={vx}, vy={vy}, wz={wz}')
        
        try:
            with self.lock:
                self.ser.write(f'M {vx} {vy} {wz}\n'.encode())
                self.ser.flush()
        except Exception as e:
            self.get_logger().error(f'Write error: {e}')

    # ── Read Serial ────────────────────────────────────────────
    def read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            with self.lock:
                available = self.ser.in_waiting
            if available == 0:
                return

            with self.lock:
                raw = self.ser.readline()

            if not raw:
                return

            line = raw.decode('utf-8', errors='ignore').strip()
            if not line:
                return

            self.get_logger().debug(f'RX: {repr(line)}')
            self.process_line(line)

        except serial.SerialException as e:
            self.get_logger().error(f'Serial error: {e}')
        except Exception as e:
            self.get_logger().debug(f'Read warn: {e}')

    # ── Process Each Line ──────────────────────────────────────
    def process_line(self, line):
        """
        Handles ALL possible ESP32 output formats:
          New format : "ENC:100,200,300,400"
          Old format : "ENC 100 200 300 400"
          Mixed      : "ENC:100 200 300 400"
        """

        # ── ENC data ──────────────────────────────────────────
        if line.upper().startswith('ENC'):
            self.enc_count += 1

            # Remove prefix — handle both ENC: and ENC<space>
            raw = line
            if ':' in raw:
                raw = raw.split(':', 1)[1]   # everything after ':'
            else:
                parts_raw = raw.split(None, 1)
                if len(parts_raw) < 2:
                    return
                raw = parts_raw[1]           # everything after 'ENC'

            # Split by comma OR space
            if ',' in raw:
                parts = raw.split(',')
            else:
                parts = raw.split()

            # Must have exactly 4 values
            if len(parts) != 4:
                self.get_logger().warn(
                    f'ENC bad format: {repr(line)}')
                return

            try:
                fl = int(parts[0].strip())
                fr = int(parts[1].strip())
                bl = int(parts[2].strip())
                br = int(parts[3].strip())
            except ValueError as e:
                self.get_logger().warn(
                    f'ENC parse error: {repr(line)} → {e}')
                return

            # Set baseline on first reading
            if self.prev_FL is None:
                self.prev_FL = fl
                self.prev_FR = fr
                self.prev_BL = bl
                self.prev_BR = br
                self.get_logger().info(
                    f'Encoder baseline: FL={fl} FR={fr} BL={bl} BR={br}')
                return

            self.compute_and_publish_odom(fl, fr, bl, br)
            return

        # ── Other messages ────────────────────────────────────
        if 'PONG' in line:
            return
        if 'READY' in line:
            self.get_logger().info('ESP32 READY - Resetting Odometry Baseline')
            self.prev_FL = None
            return
        if line.startswith('OK:'):
            return

    # ── Mecanum Odometry ───────────────────────────────────────
    def compute_and_publish_odom(self, fl, fr, bl, br):
        # Safeguard: if ESP32 resets or drops power, encoders reset to 0.
        # This prevents massive map jumps. Max physically possible delta in 0.05s is ~50 ticks.
        if abs(fl - self.prev_FL) > 300 or abs(fr - self.prev_FR) > 300:
            self.get_logger().warn('ESP32 brownout/jump detected! Resetting odometry baseline.')
            self.prev_FL = fl
            self.prev_FR = fr
            self.prev_BL = bl
            self.prev_BR = br
            return

        d_fl = (fl - self.prev_FL) * self.mpp
        d_fr = (fr - self.prev_FR) * self.mpp
        d_bl = (bl - self.prev_BL) * self.mpp
        d_br = (br - self.prev_BR) * self.mpp

        self.prev_FL = fl
        self.prev_FR = fr
        self.prev_BL = bl
        self.prev_BR = br

        # Mecanum forward kinematics
        vx    = ( d_fl + d_fr + d_bl + d_br) / 4.0
        vy    = (-d_fl + d_fr + d_bl - d_br) / 4.0
        omega = (-d_fl + d_fr - d_bl + d_br) / \
                (4.0 * (self.Lx + self.Ly))

        # Midpoint integration
        half       = self.theta + omega / 2.0
        self.x    += vx * math.cos(half) - vy * math.sin(half)
        self.y    += vx * math.sin(half) + vy * math.cos(half)
        self.theta += omega
        self.theta  = math.atan2(
            math.sin(self.theta),
            math.cos(self.theta))

        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt <= 0:
            dt = 0.1
        self.last_odom_time = now

        self.odom_count += 1
        self.publish_odom(vx / dt, vy / dt, omega / dt)

    # ── Publish Odom + TF ──────────────────────────────────────
    def publish_odom(self, vx_sec, vy_sec, omega_sec):
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

        # Odometry message
        odom = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_link'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance[0]      = 0.001
        odom.pose.covariance[7]      = 0.001
        odom.pose.covariance[35]     = 0.001
        
        odom.twist.twist.linear.x    = vx_sec
        odom.twist.twist.linear.y    = vy_sec
        odom.twist.twist.angular.z   = omega_sec
        
        self.odom_pub.publish(odom)

    # ── Status Print Every 5s ──────────────────────────────────
    def print_status(self):
        self.get_logger().info(
            f'ENC_received={self.enc_count} '
            f'odom_published={self.odom_count} '
            f'x={self.x:.3f} y={self.y:.3f} '
            f'theta={math.degrees(self.theta):.1f}deg')

        if self.enc_count == 0:
            self.get_logger().warn(
                'NO ENC data received! Check ESP32 firmware.')
        elif self.odom_count == 0:
            self.get_logger().warn(
                'ENC received but odom not published! Parsing error.')
        else:
            self.get_logger().info('Odom publishing OK ✅')

        self.enc_count  = 0
        self.odom_count = 0

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'X\n')
                self.ser.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
