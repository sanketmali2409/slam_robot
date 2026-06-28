#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

class GoalPoseBridge(Node):
    def __init__(self):
        super().__init__('goal_pose_bridge')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_cb,
            10
        )
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Goal Pose Bridge started. Waiting for /goal_pose messages...')

    def goal_cb(self, msg):
        self.get_logger().info(f'Received goal pose: x={msg.pose.position.x}, y={msg.pose.position.y}')
        
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Action server navigate_to_pose not available!')
            return
            
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg
        goal_msg.behavior_tree = ''
        
        self.get_logger().info('Sending goal to action server...')
        self.action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
