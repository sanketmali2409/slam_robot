#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')
        
        # Subscribe to transient local
        qos_sub = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, qos_sub)
        
        # Publish as volatile
        qos_pub = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map_volatile', qos_pub)
        
        self.last_map = None
        self.timer = self.create_timer(2.0, self.timer_cb)
        
    def map_cb(self, msg):
        self.last_map = msg
        
    def timer_cb(self):
        if self.last_map is not None:
            # Update timestamp
            self.last_map.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.last_map)

def main(args=None):
    rclpy.init(args=args)
    node = MapRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
