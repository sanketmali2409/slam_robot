#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')
        self.subscription = self.create_subscription(
            String,
            '/save_map',
            self.listener_callback,
            10)
        self.get_logger().info('Map Saver Node Started. Listening to /save_map')

    def listener_callback(self, msg):
        map_name = msg.data.strip()
        if not map_name:
            map_name = 'my_map'
        
        # Make sure maps directory exists
        os.system('mkdir -p ~/maps')
        
        self.get_logger().info(f'Saving map as: {map_name}')
        # Run the nav2 map saver CLI command
        result = os.system(f'ros2 run nav2_map_server map_saver_cli -f ~/maps/{map_name}')
        
        if result == 0:
            self.get_logger().info(f'✅ Map {map_name} saved successfully!')
        else:
            self.get_logger().error(f'❌ Failed to save map {map_name}.')

def main(args=None):
    rclpy.init(args=args)
    map_saver_node = MapSaverNode()
    rclpy.spin(map_saver_node)
    map_saver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
