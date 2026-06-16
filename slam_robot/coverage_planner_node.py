#!/usr/bin/env python3
"""
Coverage Planner Node — ROS 2 Jazzy
=====================================
1. Subscribes to /map (OccupancyGrid)
2. Extracts free space cells
3. Generates boustrophedon (zigzag) coverage path
4. Sends waypoints to Nav2 via NavigateToPose action
5. Supports holonomic mecanum motion (strafing between rows)

Topics:
  /map                    ← OccupancyGrid (from SLAM Toolbox)
  /coverage/path          → Path (visualization in RViz2)
  /coverage/status        → String (current status)

Actions:
  /navigate_to_pose       → Nav2 goal sender
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

import numpy as np
import math
import time


class CoveragePlannerNode(Node):
    def __init__(self):
        super().__init__('coverage_planner_node')

        # ── Parameters ──────────────────────────────────────────
        self.declare_parameter('robot_radius',      0.20)   # meters
        self.declare_parameter('coverage_overlap',  0.10)   # meters overlap between rows
        self.declare_parameter('row_direction',     'x')    # 'x' or 'y' zigzag axis
        self.declare_parameter('free_threshold',    50)     # occupancy < this = free
        self.declare_parameter('goal_timeout',      30.0)   # seconds per waypoint
        self.declare_parameter('linear_speed',      0.2)    # m/s
        self.declare_parameter('auto_start',        False)  # auto start on map receive

        self.robot_radius     = self.get_parameter('robot_radius').value
        self.overlap          = self.get_parameter('coverage_overlap').value
        self.row_direction    = self.get_parameter('row_direction').value
        self.free_threshold   = self.get_parameter('free_threshold').value
        self.goal_timeout     = self.get_parameter('goal_timeout').value
        self.linear_speed     = self.get_parameter('linear_speed').value
        self.auto_start       = self.get_parameter('auto_start').value

        # Row spacing = diameter - overlap
        self.row_spacing = (self.robot_radius * 2.0) - self.overlap

        # ── State ────────────────────────────────────────────────
        self.map_data        = None
        self.map_info        = None
        self.coverage_path   = []
        self.current_goal    = 0
        self.is_running      = False
        self.map_received    = False

        # ── Publishers ───────────────────────────────────────────
        self.path_pub   = self.create_publisher(Path,   '/coverage/path',   10)
        self.status_pub = self.create_publisher(String, '/coverage/status', 10)
        self.cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',         10)

        # ── Subscribers ──────────────────────────────────────────
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        # ── Nav2 Action Client ───────────────────────────────────
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # ── Timer for status publishing ──────────────────────────
        self.status_timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Coverage Planner Node started!')
        self.get_logger().info(f'Robot radius:  {self.robot_radius}m')
        self.get_logger().info(f'Row spacing:   {self.row_spacing:.3f}m')
        self.get_logger().info(f'Row direction: {self.row_direction}-axis zigzag')
        self.get_logger().info('Waiting for /map topic...')
        self.get_logger().info('Call service or set auto_start:=true to begin')

    # ════════════════════════════════════════════════════════════
    #  MAP CALLBACK
    # ════════════════════════════════════════════════════════════
    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

        if not self.map_received:
            self.map_received = True
            self.get_logger().info(
                f'Map received: {msg.info.width}x{msg.info.height} cells '
                f'@ {msg.info.resolution:.3f}m/cell')
            self.get_logger().info(
                f'Map size: {msg.info.width * msg.info.resolution:.1f}m x '
                f'{msg.info.height * msg.info.resolution:.1f}m')

            if self.auto_start:
                self.get_logger().info('Auto-start enabled — planning coverage...')
                self.start_coverage()

    # ════════════════════════════════════════════════════════════
    #  FREE SPACE EXTRACTION
    # ════════════════════════════════════════════════════════════
    def extract_free_space(self):
        """
        Returns binary grid where True = free navigable cell.
        Inflates obstacles by robot_radius to ensure safe paths.
        """
        if self.map_data is None:
            return None

        h, w = self.map_data.shape
        res  = self.map_info.resolution

        # Free = occupancy between 0 and free_threshold
        free = (self.map_data >= 0) & (self.map_data < self.free_threshold)

        # Inflate obstacles by robot_radius (in cells)
        inflation_cells = int(math.ceil(self.robot_radius / res))
        obstacle_mask   = ~free

        inflated = np.copy(obstacle_mask)
        for dy in range(-inflation_cells, inflation_cells + 1):
            for dx in range(-inflation_cells, inflation_cells + 1):
                if dx*dx + dy*dy <= inflation_cells*inflation_cells:
                    shifted = np.roll(np.roll(obstacle_mask, dy, axis=0), dx, axis=1)
                    inflated |= shifted

        safe_free = free & ~inflated

        free_count = np.sum(safe_free)
        self.get_logger().info(
            f'Free space: {free_count} cells '
            f'({free_count * res * res:.1f} m²)')

        return safe_free

    # ════════════════════════════════════════════════════════════
    #  BOUSTROPHEDON PATH GENERATION
    # ════════════════════════════════════════════════════════════
    def generate_boustrophedon_path(self, free_grid):
        """
        Generates zigzag (boustrophedon) coverage path.
        Alternates direction each row for efficiency.
        Supports both x-axis and y-axis sweep directions.
        Returns list of (world_x, world_y, yaw) tuples.
        """
        res    = self.map_info.resolution
        origin = self.map_info.origin.position
        h, w   = free_grid.shape

        row_cells = int(self.row_spacing / res)
        if row_cells < 1:
            row_cells = 1

        waypoints = []

        if self.row_direction == 'x':
            # Sweep along X axis, rows move in Y direction
            row_indices = range(0, h, row_cells)
            for row_num, row_y in enumerate(row_indices):
                # Find free cells in this row
                free_cols = [col for col in range(w) if free_grid[row_y, col]]
                if not free_cols:
                    continue

                # Alternate direction each row (boustrophedon)
                if row_num % 2 == 0:
                    sweep_cols = free_cols          # left to right
                    yaw = 0.0                        # facing +X
                else:
                    sweep_cols = free_cols[::-1]    # right to left
                    yaw = math.pi                    # facing -X

                # Add waypoints with spacing (not every cell)
                step = max(1, int(self.row_spacing / res))
                sampled = sweep_cols[::step]
                if sweep_cols[-1] not in sampled:
                    sampled.append(sweep_cols[-1])

                for col in sampled:
                    wx = origin.x + (col + 0.5) * res
                    wy = origin.y + (row_y + 0.5) * res
                    waypoints.append((wx, wy, yaw))

        else:
            # Sweep along Y axis, rows move in X direction
            col_indices = range(0, w, row_cells)
            for col_num, col_x in enumerate(col_indices):
                free_rows = [row for row in range(h) if free_grid[row, col_x]]
                if not free_rows:
                    continue

                if col_num % 2 == 0:
                    sweep_rows = free_rows
                    yaw = math.pi / 2.0    # facing +Y
                else:
                    sweep_rows = free_rows[::-1]
                    yaw = -math.pi / 2.0   # facing -Y

                step = max(1, int(self.row_spacing / res))
                sampled = sweep_rows[::step]
                if sweep_rows[-1] not in sampled:
                    sampled.append(sweep_rows[-1])

                for row in sampled:
                    wx = origin.x + (col_x + 0.5) * res
                    wy = origin.y + (row + 0.5) * res
                    waypoints.append((wx, wy, yaw))

        self.get_logger().info(
            f'Generated {len(waypoints)} waypoints '
            f'covering {len(waypoints) * self.row_spacing:.1f}m path length')

        return waypoints

    # ════════════════════════════════════════════════════════════
    #  BUILD ROS PATH MESSAGE (for RViz2 visualization)
    # ════════════════════════════════════════════════════════════
    def build_path_msg(self, waypoints):
        path_msg             = Path()
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        for (wx, wy, yaw) in waypoints:
            pose              = PoseStamped()
            pose.header       = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

        return path_msg

    # ════════════════════════════════════════════════════════════
    #  MECANUM STRAFE BETWEEN ROWS
    # ════════════════════════════════════════════════════════════
    def strafe_to_next_row(self, duration_sec=1.5):
        """
        Uses mecanum holonomic motion to strafe sideways
        when transitioning between coverage rows.
        This is the key advantage of mecanum over differential drive.
        """
        twist              = Twist()
        twist.linear.y     = self.linear_speed   # strafe right
        twist.linear.x     = 0.0
        twist.angular.z    = 0.0

        self.get_logger().info('Mecanum strafe: moving to next row...')
        start = time.time()
        rate  = self.create_rate(10)

        while time.time() - start < duration_sec:
            self.cmd_pub.publish(twist)
            rate.sleep()

        # Stop
        self.cmd_pub.publish(Twist())

    # ════════════════════════════════════════════════════════════
    #  SEND SINGLE WAYPOINT TO NAV2
    # ════════════════════════════════════════════════════════════
    def send_goal(self, wx, wy, yaw):
        """Send a single pose goal to Nav2 NavigateToPose action."""

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = wx
        goal_msg.pose.pose.position.y = wy
        goal_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        # Send goal and wait for result
        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback)

        rclpy.spin_until_future_complete(
            self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(f'Goal rejected at ({wx:.2f}, {wy:.2f})')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=self.goal_timeout)

        result = result_future.result()
        if result:
            self.get_logger().info(f'Reached ({wx:.2f}, {wy:.2f}) ✅')
            return True
        else:
            self.get_logger().warn(f'Failed to reach ({wx:.2f}, {wy:.2f}) ⚠️')
            return False

    def nav_feedback_callback(self, feedback_msg):
        """Log distance remaining to current goal."""
        dist = feedback_msg.feedback.distance_remaining
        if dist > 0.3:
            self.get_logger().debug(f'Distance remaining: {dist:.2f}m')

    # ════════════════════════════════════════════════════════════
    #  MAIN COVERAGE EXECUTION
    # ════════════════════════════════════════════════════════════
    def start_coverage(self):
        """Full coverage execution pipeline."""

        if not self.map_received:
            self.get_logger().error('No map received yet! Run SLAM first.')
            return

        if self.is_running:
            self.get_logger().warn('Coverage already running!')
            return

        self.is_running   = True
        self.current_goal = 0

        self.publish_status_msg('PLANNING')
        self.get_logger().info('=' * 50)
        self.get_logger().info('Starting full area coverage...')

        # Step 1: Extract free space
        free_grid = self.extract_free_space()
        if free_grid is None or np.sum(free_grid) == 0:
            self.get_logger().error('No free space found in map!')
            self.is_running = False
            return

        # Step 2: Generate boustrophedon path
        self.coverage_path = self.generate_boustrophedon_path(free_grid)
        if not self.coverage_path:
            self.get_logger().error('Could not generate coverage path!')
            self.is_running = False
            return

        # Step 3: Publish path for RViz2 visualization
        path_msg = self.build_path_msg(self.coverage_path)
        self.path_pub.publish(path_msg)
        self.get_logger().info('Coverage path published to /coverage/path')
        self.get_logger().info('Add "Path" display in RViz2 to see it')

        # Step 4: Execute waypoints
        self.publish_status_msg('EXECUTING')
        total    = len(self.coverage_path)
        prev_yaw = None

        for i, (wx, wy, yaw) in enumerate(self.coverage_path):
            if not self.is_running:
                self.get_logger().info('Coverage cancelled!')
                break

            self.current_goal = i
            self.get_logger().info(
                f'Waypoint {i+1}/{total}: '
                f'({wx:.2f}, {wy:.2f}) yaw={math.degrees(yaw):.0f}°')

            # Mecanum strafe when changing rows (yaw flips)
            if prev_yaw is not None and abs(yaw - prev_yaw) > 0.1:
                self.strafe_to_next_row(duration_sec=1.0)

            # Send goal to Nav2
            success = self.send_goal(wx, wy, yaw)
            if not success:
                self.get_logger().warn(f'Skipping waypoint {i+1}, continuing...')

            prev_yaw = yaw

            # Re-publish path with remaining waypoints highlighted
            remaining = self.build_path_msg(self.coverage_path[i:])
            self.path_pub.publish(remaining)

        # Step 5: Done
        self.is_running = False
        if self.current_goal >= total - 1:
            self.publish_status_msg('COMPLETED')
            self.get_logger().info('=' * 50)
            self.get_logger().info('✅ Full area coverage COMPLETED!')
            self.get_logger().info(f'   Visited {total} waypoints')
            self.get_logger().info(
                f'   Area covered: ~{total * self.row_spacing * self.row_spacing:.1f} m²')
        else:
            self.publish_status_msg('CANCELLED')

    def stop_coverage(self):
        """Stop coverage execution."""
        self.is_running = False
        self.cmd_pub.publish(Twist())
        self.get_logger().info('Coverage stopped!')
        self.publish_status_msg('STOPPED')

    def publish_status(self):
        """Publish status periodically."""
        if self.is_running:
            total = len(self.coverage_path)
            msg   = f'RUNNING: {self.current_goal+1}/{total} waypoints'
            self.publish_status_msg(msg)

    def publish_status_msg(self, text):
        msg      = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_coverage()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
