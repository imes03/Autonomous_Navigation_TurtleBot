#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class Explorer(Node):

    def __init__(self):
        super().__init__('explorer_node')

        self.navigator = BasicNavigator()

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        self.map_data = None
        self.map_info = None

        self.timer = self.create_timer(5.0, self.explore)

        self.get_logger().info("Explorer node started")

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_info = msg.info

    def find_frontiers(self):
        frontiers = []

        for y in range(1, self.map_data.shape[0] - 1):
            for x in range(1, self.map_data.shape[1] - 1):

                if self.map_data[y][x] == 0:  # free space

                    neighbors = [
                        self.map_data[y+1][x],
                        self.map_data[y-1][x],
                        self.map_data[y][x+1],
                        self.map_data[y][x-1]
                    ]

                    if -1 in neighbors:  # unknown nearby
                        frontiers.append((x, y))

        return frontiers

    def grid_to_world(self, x, y):
        wx = x * self.map_info.resolution + self.map_info.origin.position.x
        wy = y * self.map_info.resolution + self.map_info.origin.position.y
        return wx, wy

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.navigator.goToPose(goal)
        self.get_logger().info(f"Going to: {x:.2f}, {y:.2f}")

    def explore(self):
        if self.map_data is None:
            return

        frontiers = self.find_frontiers()

        if not frontiers:
            self.get_logger().info("No frontiers left. Exploration complete.")
            return

        # Pick random frontier (simple strategy)
        target = frontiers[np.random.randint(len(frontiers))]
        wx, wy = self.grid_to_world(target[0], target[1])

        self.send_goal(wx, wy)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()