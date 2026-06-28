import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('send_goal_node')
    publisher = node.create_publisher(PoseStamped, '/goal_pose', 10)
    
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.pose.position.x = 2.0
    msg.pose.position.y = 2.0
    msg.pose.orientation.w = 1.0
    
    # Wait for subscribers
    import time
    time.sleep(2)
    
    publisher.publish(msg)
    node.get_logger().info('Published goal to /goal_pose')
    
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
