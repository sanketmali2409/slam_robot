#!/usr/bin/env python3
"""
CMD_VEL to Serial Node
-----------------------
Subscribes to /cmd_vel (from teleop keyboard).
Converts linear.x and angular.z into simple serial commands.
Sends those commands to ESP32 over UART.

Serial protocol (simple text):
  "F 150 150\n"   = Forward, left_speed=150, right_speed=150
  "B 150 150\n"   = Backward
  "L 100 150\n"   = Turn left (left slower)
  "R 150 100\n"   = Turn right (right slower)
  "S 0 0\n"       = Stop

ESP32 reads these strings and drives L293D accordingly.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time


class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_serial_node')

        # --- Parameters (change port if needed) ---
        self.declare_parameter('serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_speed', 200)  # 0-255 PWM range

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            time.sleep(2)  # Wait for ESP32 to reset after serial connect
            self.get_logger().info(f'Serial connected: {port} @ {baud} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            self.get_logger().error('Check: ls /dev/ttyUSB* or /dev/ttyACM*')
            self.ser = None

        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info('Listening to /cmd_vel...')

    def cmd_vel_callback(self, msg):
        if self.ser is None:
            return

        linear = msg.linear.x    # positive = forward
        angular = msg.angular.z  # positive = turn left

        # Convert to left/right wheel speeds
        left_speed  = linear - angular * 0.5
        right_speed = linear + angular * 0.5

        # Normalize to PWM range
        max_val = max(abs(left_speed), abs(right_speed), 1.0)
        left_pwm  = int((left_speed / max_val) * self.max_speed)
        right_pwm = int((right_speed / max_val) * self.max_speed)

        # Determine direction
        if linear > 0.05:
            direction = 'F'
        elif linear < -0.05:
            direction = 'B'
            left_pwm  = abs(left_pwm)
            right_pwm = abs(right_pwm)
        elif angular > 0.05:
            direction = 'L'
            left_pwm  = abs(left_pwm)
            right_pwm = abs(right_pwm)
        elif angular < -0.05:
            direction = 'R'
            left_pwm  = abs(left_pwm)
            right_pwm = abs(right_pwm)
        else:
            direction = 'S'
            left_pwm  = 0
            right_pwm = 0

        command = f'{direction} {left_pwm} {right_pwm}\n'
        try:
            self.ser.write(command.encode())
            self.get_logger().debug(f'Sent: {command.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def destroy_node(self):
        # Stop motors when node shuts down
        if self.ser and self.ser.is_open:
            self.ser.write(b'S 0 0\n')
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
