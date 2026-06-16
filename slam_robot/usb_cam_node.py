#!/usr/bin/env python3
"""
USB Camera Node — ROS 2 Jazzy
===============================
Captures frames from USB webcam and publishes:
  /camera/image_raw        (sensor_msgs/Image)
  /camera/camera_info      (sensor_msgs/CameraInfo)

Requirements:
  sudo apt install ros-jazzy-cv-bridge
  pip3 install opencv-python --break-system-packages

Usage:
  ros2 run slam_robot usb_cam_node
  ros2 run slam_robot usb_cam_node --ros-args -p device_id:=0

Display in RViz2:
  Add → By Topic → /camera/image_raw → Image
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import cv2

try:
    from cv_bridge import CvBridge
    CV_BRIDGE = True
except ImportError:
    CV_BRIDGE = False


class UsbCamNode(Node):
    def __init__(self):
        super().__init__('usb_cam_node')

        # ── Parameters ────────────────────────────────────────
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width',     640)
        self.declare_parameter('height',    480)
        self.declare_parameter('fps',       30)
        self.declare_parameter('frame_id',  'camera_link')

        device_id     = self.get_parameter('device_id').value
        self.width    = self.get_parameter('width').value
        self.height   = self.get_parameter('height').value
        fps           = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value

        if not CV_BRIDGE:
            self.get_logger().error('cv_bridge not found!')
            self.get_logger().error(
                'Run: sudo apt install ros-jazzy-cv-bridge')
            return

        # ── Open camera ────────────────────────────────────────
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open /dev/video{device_id}')
            self.get_logger().error('Check: ls /dev/video*')
            return

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS,          fps)

        w   = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h   = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps_actual = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f'Camera /dev/video{device_id} opened ✅')
        self.get_logger().info(
            f'Resolution: {w}x{h} @ {fps_actual:.0f} FPS')

        # ── Bridge and publishers ──────────────────────────────
        self.bridge   = CvBridge()
        self.img_pub  = self.create_publisher(
            Image,      '/camera/image_raw',   10)
        self.info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 10)

        # ── Timer ─────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / fps,
                                       self.capture_and_publish)
        self.get_logger().info(
            f'Publishing /camera/image_raw at {fps} Hz ✅')

    def capture_and_publish(self):
        if not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return

        now = self.get_clock().now().to_msg()

        # Publish image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(
                frame, encoding='bgr8')
            img_msg.header.stamp    = now
            img_msg.header.frame_id = self.frame_id
            self.img_pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'Publish error: {e}')
            return

        # Publish camera info
        info             = CameraInfo()
        info.header.stamp    = now
        info.header.frame_id = self.frame_id
        info.width           = self.width
        info.height          = self.height
        info.distortion_model = 'plumb_bob'
        fx = fy = float(self.width) * 0.8
        cx = float(self.width)  / 2.0
        cy = float(self.height) / 2.0
        info.k = [fx,  0.0, cx,
                  0.0, fy,  cy,
                  0.0, 0.0, 1.0]
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx,  0.0, cx,  0.0,
                  0.0, fy,  cy,  0.0,
                  0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info('Camera released')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
