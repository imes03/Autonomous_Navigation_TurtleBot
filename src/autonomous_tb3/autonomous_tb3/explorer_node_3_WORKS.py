#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np
import math
import tf2_ros
from tf2_ros import TransformException


class Explorer(Node):

    def __init__(self):
        super().__init__('explorer_node')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Data
        self.map_data = None
        self.map_info = None
        self.front_distance = float('inf')

        self.current_goal = None
        self.failed_goals = []

        #  NEW: mode system
        self.mode = "BOOTSTRAP"

        # Timer
        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("Explorer (frontier + safe control) started")

    # ---------------- SENSOR ----------------
    def scan_callback(self, msg):
        center = len(msg.ranges) // 2
        self.front_distance = msg.ranges[center]

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    # ---------------- NEW ----------------
    def map_is_ready(self):
        if self.map_data is None:
            return False

        known_cells = np.count_nonzero(self.map_data != -1)
        total_cells = self.map_data.size

        # Avoid division issues
        if total_cells == 0:
            return False

        return (known_cells / total_cells) > 0.05

    # ---------------- YOUR FUNCTIONS (UNCHANGED) ----------------
    def find_frontiers(self):
        frontiers = []
        for y in range(1, self.map_data.shape[0] - 1):
            for x in range(1, self.map_data.shape[1] - 1):
                if self.map_data[y][x] == 0:
                    neighbors = [
                        self.map_data[y+1][x],
                        self.map_data[y-1][x],
                        self.map_data[y][x+1],
                        self.map_data[y][x-1]
                    ]
                    if -1 in neighbors:
                        frontiers.append((x, y))
        return frontiers

    def grid_to_world(self, x, y):
        wx = x * self.map_info.resolution + self.map_info.origin.position.x
        wy = y * self.map_info.resolution + self.map_info.origin.position.y
        return wx, wy

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )
        except TransformException:
            return None

    def filter_failed_frontiers(self, frontiers):
        filtered = []
        for f in frontiers:
            wx, wy = self.grid_to_world(f[0], f[1])
            if not any(math.hypot(wx - fg[0], wy - fg[1]) < 0.7 for fg in self.failed_goals):
                filtered.append(f)
        return filtered

    def select_best_frontier(self, frontiers):
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None

        rx, ry = robot_pos
        best = None
        min_dist = float('inf')

        for f in frontiers:
            wx, wy = self.grid_to_world(f[0], f[1])
            dist = math.hypot(wx - rx, wy - ry)

            if 0.5 < dist < min_dist:
                min_dist = dist
                best = (wx, wy)

        return best

    # ---------------- CONTROL ----------------
    def control_loop(self):

        if self.map_data is None:
            return

        twist = Twist()

        # ================= BOOTSTRAP =================
        if self.mode == "BOOTSTRAP":

            if self.map_is_ready():
                self.get_logger().info("Map ready switching to EXPLORATION")
                self.mode = "EXPLORE"
                return

            # Safe exploration to build map
            if self.front_distance < 0.6:
                twist.linear.x = 0.0
                twist.angular.z = 0.6
            elif self.front_distance < 1.0:
                twist.linear.x = 0.05
                twist.angular.z = 0.3
            else:
                twist.linear.x = 0.15
                twist.angular.z = 0.1

            self.cmd_pub.publish(twist)
            return

        # ================= EXPLORATION =================
        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return

        # Select goal if none
        if self.current_goal is None:
            frontiers = self.find_frontiers()
            frontiers = self.filter_failed_frontiers(frontiers)

            if not frontiers:
                self.get_logger().info("No frontiers found fallback to BOOTSTRAP")
                self.mode = "BOOTSTRAP"
                return

            best = self.select_best_frontier(frontiers)
            if best:
                self.current_goal = best
                self.get_logger().info(f"New goal: {best}")

        # Move towards goal
        gx, gy = self.current_goal
        rx, ry = robot_pos

        dx = gx - rx
        dy = gy - ry
        distance = math.hypot(dx, dy)
 #       angle_to_goal = math.atan2(dy, dx)
        angle_to_goal = math.atan2(dy, dx)

        # Get robot orientation (yaw)
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())
            q = transform.transform.rotation

            # Convert quaternion → yaw
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        except:
            return

        # Compute angular error
        angle_error = angle_to_goal - robot_yaw

        # Normalize angle [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # CONTROL LOGIC
        if abs(angle_error) > 0.3:
            # rotate first
            twist.linear.x = 0.0
            twist.angular.z = 0.6 * angle_error
        else:
            # go forward
            twist.linear.x = min(0.2, distance)
            twist.angular.z = 0.3 * angle_error



        # Safety: obstacle ahead
        if self.front_distance < 0.5:
            twist.angular.z = 0.6
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)
            return

        # Smooth proportional control
 #       twist.linear.x = min(0.2, distance)
 #       twist.angular.z = 1.2 * angle_to_goal  # reduced gain (more stable)

        self.cmd_pub.publish(twist)

        # Goal reached
        if distance < 0.3:
            self.get_logger().info("Goal reached")
            self.current_goal = None


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()