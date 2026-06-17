#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult
from apriltag_msgs.msg import AprilTagDetectionArray
from rclpy.duration import Duration

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from nav_msgs.msg import OccupancyGrid

import numpy as np
import math
import random

import tf2_ros
from tf2_ros import TransformException


class RRTNode:

    def __init__(self, x, y, parent=None):

        self.x = x
        self.y = y
        self.parent = parent


class Explorer(Node):

    def __init__(self):

        super().__init__('explorer_node')
        self.start_time = self.get_clock().now()
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # RRT TREE VISUALIZATION
        self.rrt_marker_pub = self.create_publisher(
            Marker,
            '/rrt_tree',
            10
        )

        # SELECTED GOAL VISUALIZATION
        self.goal_marker_pub = self.create_publisher(
            Marker,
            '/rrt_goal',
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.navigator = BasicNavigator()

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.tag_callback,
            10
        )

        self.map_data = None
        self.map_info = None

        self.timer = self.create_timer(
            3.0,
            self.explore
        )

        self.visual_timer = self.create_timer(
            0.1,
            self.visual_servo
        )
        
        self.current_goal = None
        self.exploring = False
        self.failed_goals = []
        self.last_goal = None

        # Apriltag 
        self.tag_detected = False
        self.tag_pending = False
        self.tag_goal_sent = False
        self.approaching_tag = False
        self.tag_position = None
        self.mission_complete = False
        self.saved_goal_x = None
        self.saved_goal_y = None
        self.saved_goal_yaw = None
        self.state = "EXPLORING"
        self.tag_x = None
        self.tag_y = None
        self.tag_center_x = None
        self.tag_size = None
        self.tag_x = None
        self.tag_y = None
        self.desired_distance = 0.60
        self.centered_tag = False
        self.desired_distance = 0.40
        self.tag_center_x = None
        self.tag_size = None

        # RRT PARAMETERS
        self.rrt_iterations = 500
        self.rrt_step_size = 0.6                    #small maze
        self.frontier_search_radius = 2.0        
        self.optimistic_distance = 3
        self.get_logger().info(
            'Debug: Explorer node started'
        )

    def map_callback(self, msg):

        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )

        self.map_info = msg.info

    def tag_callback(self, msg):

        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]

        self.tag_center_x = detection.centre.x

        c = detection.corners

        s1 = math.hypot(
            c[0].x-c[1].x,
            c[0].y-c[1].y
        )

        s2 = math.hypot(
            c[1].x-c[2].x,
            c[1].y-c[2].y
        )

        s3 = math.hypot(
            c[2].x-c[3].x,
            c[2].y-c[3].y
        )

        s4 = math.hypot(
            c[3].x-c[0].x,
            c[3].y-c[0].y
        )

        self.tag_size = (s1+s2+s3+s4)/4

        try:

            tf = self.tf_buffer.lookup_transform(
                'base_link',
                'tag36h11:0',
                rclpy.time.Time()
            )

            self.tag_x = tf.transform.translation.x
            self.tag_y = tf.transform.translation.y

        except TransformException:
            pass

        

        self.get_logger().info(
            f"Tag center={self.tag_center_x:.1f} size={self.tag_size:.1f}"
        )
        tag_id = detection.id

        self.get_logger().info(
            f'APRILTAG DETECTED! >>>>>>>>>>> Tag:   {tag_id}'
        )

        # STOP CURRENT NAVIGATION
        if not self.tag_detected and not self.tag_pending:

            self.tag_pending = True

            self.get_logger().info(
                "AprilTag seen. Will finish current goal first."
            )





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

        wx = x * self.map_info.resolution + \
            self.map_info.origin.position.x

        wy = y * self.map_info.resolution + \
            self.map_info.origin.position.y

        return wx, wy

    def world_to_grid(self, wx, wy):

        gx = int(
            (wx - self.map_info.origin.position.x)
            / self.map_info.resolution
        )

        gy = int(
            (wy - self.map_info.origin.position.y)
            / self.map_info.resolution
        )

        return gx, gy
    def in_bounds(self, gx, gy):

        if gx < 0 or gy < 0:
            return False

        if gx >= self.map_data.shape[1]:
            return False

        if gy >= self.map_data.shape[0]:
            return False

        return True

    def get_robot_position(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x

            y = transform.transform.translation.y

            return x, y

        except TransformException:

            return None


    # FILTER FAILED FRONTIERS

    def filter_failed_frontiers(self, frontiers):

        filtered = []
        for f in frontiers:

            wx, wy = self.grid_to_world(f[0], f[1])

            skip = False

            for fg in self.failed_goals:

                if math.hypot(
                    wx - fg[0],
                    wy - fg[1]
                ) < 0.2:

                    skip = True
                    break

            if not skip:

                filtered.append(f)

        return filtered

    # CHECK FREE SPACE

    def is_free(self, wx, wy):
        if self.map_data is None:
            return False
        
        if self.mission_complete:
            self.cmd_pub.publish(Twist())
            return

        gx, gy = self.world_to_grid(wx, wy)

        if gx < 0 or gy < 0:
            return False

        if gx >= self.map_data.shape[1]:
            return False

        if gy >= self.map_data.shape[0]:
            return False

        value = self.map_data[gy][gx]

        # DEFINITELY FREE

        if value == 0:
            return True

        # OCCUPIED

        if value > 50:
            return False

        # OPTIMISTIC UNKNOWN EXPLORATION

        if value == -1:

            robot_pos = self.get_robot_position()

            if robot_pos is None:
                return False

            rx, ry = robot_pos

            dist = math.hypot(
                wx - rx,
                wy - ry
            )

            # allow nearby unknown space
            if dist < self.optimistic_distance:

                return True

        return False

    # RANDOM SAMPLE
    def sample_random_point(self):
        width = self.map_data.shape[1]                  # map rows
        height = self.map_data.shape[0]                 # map columns 
        while True:
            gx = random.randint(0, width - 1)           # Random x
            gy = random.randint(0, height - 1)          # Random y
            cell = self.map_data[gy][gx]                # Random cell of the ocupancy grid

            if cell == 0 or cell == -1:                 #could be free or unknown but not Obstacles
                wx, wy = self.grid_to_world(gx, gy)     #Coordenate in real world
                return wx, wy

    # NEAREST TREE NODE
    def nearest_node(self, tree, rand_x, rand_y):       #tree = all existing nodes RRT
        nearest = tree[0]                               #Initialize first = nearest
        min_dist = float('inf')                         #any distance
        for node in tree:
            dist = math.hypot(                          #nearest distance (straight line->hypothenuse)      
                rand_x - node.x,
                rand_y - node.y
            )
            if dist < min_dist:                         #Compare current with best node
                min_dist = dist                         #save new best node
                nearest = node
        return nearest

    # STEER TOWARD RANDOM POINT
    def steer(self, nearest, rand_x, rand_y):           # nearest node in tree to random x,y
        theta = math.atan2(                             # angle from current node to random x,y
            rand_y - nearest.y,
            rand_x - nearest.x
        )
        new_x = nearest.x + \
            self.rrt_step_size * math.cos(theta)        #new horizontal componnent x
        new_y = nearest.y + \
            self.rrt_step_size * math.sin(theta)        #new vertical componnent x
        
        return RRTNode(                                         
            new_x,                                      #Call class create node connected to parent (nearest) in x,y
            new_y,
            nearest
        )

    # COLLISION CHECK

    def collision_free(self, node):
        return self.is_free(
            node.x,
            node.y
        )
    # EDGE COLLISION CHECK

    def edge_collision_free(self, x1, y1, x2, y2):

        dist = math.hypot(
            x2 - x1,
            y2 - y1
        )

        steps = int(
            dist / (self.map_info.resolution * 0.5)
        )

        if steps == 0:
            return True

        for i in range(steps + 1):

            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if not self.is_free(x, y):

                return False

        return True
    
    def has_clearance(self, x, y, radius=0.4):

        steps = int(radius / self.map_info.resolution)

        gx, gy = self.world_to_grid(x, y)

        for dx in range(-steps, steps + 1):

            for dy in range(-steps, steps + 1):

                nx = gx + dx
                ny = gy + dy

                if not self.in_bounds(nx, ny):
                    return False

                cell = self.map_data[ny][nx]

                if cell > 50:
                    return False

        return True

    # FRONTIER CHECK

    def is_frontier_point(self, wx, wy):

        gx, gy = self.world_to_grid(wx, wy)

        if gx <= 0 or gy <= 0:
            return False

        if gx >= self.map_data.shape[1] - 1:
            return False

        if gy >= self.map_data.shape[0] - 1:
            return False

        if self.map_data[gy][gx] != 0:
            return False

        neighbors = [

            self.map_data[gy+1][gx],
            self.map_data[gy-1][gx],
            self.map_data[gy][gx+1],
            self.map_data[gy][gx-1],

            self.map_data[gy+1][gx+1],
            self.map_data[gy+1][gx-1],
            self.map_data[gy-1][gx+1],
            self.map_data[gy-1][gx-1]

        ]


        return -1 in neighbors

    # INFORMATION GAIN

    def information_gain(self, wx, wy):

        gx, gy = self.world_to_grid(wx, wy)

        radius_cells = int(
            self.frontier_search_radius /
            self.map_info.resolution
        )

        unknown_count = 0

        for dy in range(-radius_cells, radius_cells):

            for dx in range(-radius_cells, radius_cells):
                nx = gx + dx
                ny = gy + dy

                if nx < 0 or ny < 0:
                    continue

                if nx >= self.map_data.shape[1]:
                    continue

                if ny >= self.map_data.shape[0]:
                    continue

                if self.map_data[ny][nx] == -1:

                    unknown_count += 1

        return unknown_count

    # PUBLISH RRT TREE

    def publish_rrt_tree(self, tree):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'rrt_tree'
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.03

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for node in tree:

            if node.parent is None:
                continue

            p1 = Point()
            p1.x = node.x
            p1.y = node.y
            p1.z = 0.0

            p2 = Point()
            p2.x = node.parent.x
            p2.y = node.parent.y
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

        self.rrt_marker_pub.publish(marker)

    # PUBLISH SELECTED GOAL

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'goal'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_marker_pub.publish(marker)

    # SMART RRT FRONTIER SELECTION

    def select_rrt_frontier(self):

        robot_pos = self.get_robot_position()
        if robot_pos is None:
            return None

        rx, ry = robot_pos
        root = RRTNode(rx, ry)
        tree = [root]
        candidates = []

        self.get_logger().info(
            'Running RRT exploration'
        )

        for i in range(self.rrt_iterations):

            rand_x, rand_y = self.sample_random_point()
            nearest = self.nearest_node(
                tree,
                rand_x,
                rand_y
            )

            new_node = self.steer(
                nearest,
                rand_x,
                rand_y
            )

            if not self.edge_collision_free(
                    nearest.x,
                    nearest.y,
                    new_node.x,
                    new_node.y):
                continue

            if not self.has_clearance(
                    new_node.x,
                    new_node.y,
                    radius=0.2):
                continue

            tree.append(new_node)

            if i % 10 == 0:

                self.publish_rrt_tree(tree)

            gain = self.information_gain(
                new_node.x,
                new_node.y
            )

            distance = math.hypot(
                new_node.x - rx,
                new_node.y - ry
            )
            continuity_bonus = 0.0
            if self.last_goal is not None:

                d_last = math.hypot(
                    new_node.x - self.last_goal[0],
                    new_node.y - self.last_goal[1]
                )

                # premia candidatos próximos al objetivo anterior
                continuity_bonus = max(0, 3.0 - d_last)
         

            # Gain for deep exploration
            score = gain + (distance * 0.2)     #changed in small maze to far frontiers

            # ignore useless nodes
            if gain > 10:

                # Reject frontier goals near walls
                if not self.has_clearance(
                        new_node.x,
                        new_node.y,
                        radius=0.30):
                    continue
                # Must be an actual frontier
                if not self.is_frontier_point(
                        new_node.x,
                        new_node.y):
                    continue

                candidates.append((
                    score,
                    gain,
                    distance,
                    new_node.x,
                    new_node.y
                ))
                


        if len(candidates) == 0:

            self.get_logger().warn(
                'No frontier candidates found'
            )

            return None

        candidates.sort(
            key=lambda c: c[0],
            reverse=True
        )

        best = candidates[0]

        best_x = best[3]
        best_y = best[4]
        self.last_goal = (
            best_x,
            best_y
        )

        self.get_logger().info(
            f'Selected frontier: '
            f'({best_x:.2f}, {best_y:.2f}) '
            f'gain={best[1]} '
            f'distance={best[2]:.2f} '
            f'score={best[0]:.2f}'
        )

        self.publish_goal_marker(
            best_x,
            best_y
        )

        return (
            best_x,
            best_y
        )

    def create_pose(self, x, y):

        pose = PoseStamped()

        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0

        return pose

    def send_goal(self, x, y, yaw=0.0, tag_goal=False):

        goal = PoseStamped()

        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0) 

        self.navigator.goToPose(goal)
        self.current_goal = (x, y)

        if not tag_goal:
            self.exploring = True

        self.get_logger().info(
            f'New goal: {x:.2f}, {y:.2f}'
        )

    def explore(self):
        
        self.get_logger().info(
            f"EXPLORE: mission={self.mission_complete} "
            f"tag={self.tag_detected} "
            f"pending={self.tag_pending} "
            f"exploring={self.exploring}"
        )

        if self.map_data is None:
            return

        if self.mission_complete:
            self.cmd_pub.publish(Twist())
            return

        # Tag mode is handled by visual_servo()
        if self.tag_detected:
            return

        # BOOTSTRAP MODE
        if not self.map_is_ready():

            self.get_logger().info(
                'Bootstrapping SLAM...'
            )

            twist = Twist()
            twist.linear.x = 0.07
            twist.angular.z = 0.1
            self.cmd_pub.publish(twist)

            return

        elapsed = (
            self.get_clock().now() - self.start_time
        ).nanoseconds / 1e9

        if elapsed < 10.0:

            self.get_logger().info(
                'Waiting before starting exploration...'
            )

            return

        self.cmd_pub.publish(Twist())

        ####################################################
        # NAVIGATION STATE MACHINE
        ####################################################

        if self.exploring:

            if not self.navigator.isTaskComplete():

                robot_pos = self.get_robot_position()

                if robot_pos is None:
                    return

                rx, ry = robot_pos

                if not self.has_clearance(rx, ry, radius=0.25):

                    self.get_logger().warn(
                        'Too close to wall'
                    )

                feedback = self.navigator.getFeedback()

                if feedback is not None:

                    self.get_logger().info(
                        f"Distance remaining: {feedback.distance_remaining:.2f}"
                    )

                self.get_logger().info(
                    '1. Robot navigating to current frontier...'
                )

                return

            result = self.navigator.getResult()

            ################################################
            # GOAL REACHED
            ################################################

            if result == TaskResult.SUCCEEDED:

                self.get_logger().info(
                    'Goal achieved! Frontier reached'
                )

                self.current_goal = None
                self.exploring = False

                # AprilTag was previously detected
                if self.tag_pending:

                    self.get_logger().info(
                        "Frontier reached. Switching to tag mode."
                    )

                    self.tag_detected = True
                    self.tag_pending = False
                    self.get_logger().info(
                        "AprilTag seen. Will finish current goal first."
                    )

                    return

                # small rotation for SLAM improvement
                rotate_twist = Twist()
                rotate_twist.angular.z = 0.3

                self.cmd_pub.publish(rotate_twist)

                rclpy.spin_once(
                    self,
                    timeout_sec=0.5
                )

                self.cmd_pub.publish(Twist())

            ################################################
            # GOAL FAILED
            ################################################

            elif result == TaskResult.FAILED:

                self.get_logger().warn(
                    'Goal failed, marking as bad'
                )

                if self.current_goal:

                    self.failed_goals.append(
                        self.current_goal
                    )

                self.current_goal = None
                self.exploring = False

            ################################################
            # GOAL CANCELED
            ################################################

            elif result == TaskResult.CANCELED:

                self.get_logger().warn(
                    'Goal canceled'
                )

                self.current_goal = None
                self.exploring = False

        ####################################################
        # DON'T SEND NEW GOALS IF TAG WAS SEEN
        ####################################################

        if self.tag_pending:
            return

        ####################################################
        # FRONTIER SEARCH
        ####################################################

        frontiers = self.find_frontiers()

        if len(frontiers) < 5:

            self.get_logger().info(
                'Not enough frontiers yet...'
            )

            return

        if not frontiers:

            self.get_logger().info(
                'Exploration complete!'
            )

            return

        frontiers = self.filter_failed_frontiers(
            frontiers
        )

        if not frontiers:

            self.get_logger().warn(
                'No valid frontiers left'
            )

            return

        ####################################################
        # RRT FRONTIER SELECTION
        ####################################################

        best = self.select_rrt_frontier()

        if best is None:
            return

        robot_pos = self.get_robot_position()

        if robot_pos is None:
            return

        start = self.create_pose(
            robot_pos[0],
            robot_pos[1]
        )

        goal = self.create_pose(
            best[0],
            best[1]
        )

        path = self.navigator.getPath(
            start,
            goal
        )

        if path is None:

            self.get_logger().warn(
                "Nav2 cannot find a path to this frontier"
            )

            self.failed_goals.append(
                best
            )

            return

        # Avoid resending same goal
        if self.current_goal:

            dist = math.hypot(
                best[0] - self.current_goal[0],
                best[1] - self.current_goal[1]
            )

            if dist < 0.5:
                return

        self.send_goal(
            best[0],
            best[1]
        )   
    def visual_servo(self):

        self.get_logger().info(
            f"VISUAL SERVO: tag={self.tag_detected} "
            f"mission={self.mission_complete} "
            f"exploring={self.exploring}"
        )

        if not self.tag_detected:
            self.get_logger().info("EXIT 1")
            return

        if self.mission_complete:
            self.get_logger().info("EXIT 2")

            return
        self.get_logger().info("ENTER TF")
        
        # only run after exploration has stopped
        if self.exploring:
            self.get_logger().info("EXIT 3")
            return
        

        try:

            tf = self.tf_buffer.lookup_transform(
                'base_link',
                'tag36h11:0',
                rclpy.time.Time()
            )

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y

            self.get_logger().info(
                f"tx={tx:.2f}, ty={ty:.2f}, center={self.tag_center_x}"
            )

        except TransformException as e:

            self.get_logger().warn(
                f"TF ERROR: {e}"
            )
            self.cmd_pub.publish(Twist())
            return

        distance = math.sqrt(tx*tx + ty*ty)

        twist = Twist()

        image_center = 160

        error = image_center - self.tag_center_x
        self.get_logger().info(
            f"error={error:.1f}"
        )
        # CENTER TAG
        if abs(error) > 20:

            twist.angular.z = 0.003 * error

            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"Centering error={error:.1f}"
            )

            return

        # APPROACH TAG
        self.get_logger().info(
            f"distance={distance:.2f}"
        )
        if distance > 0.60:

            twist.linear.x = 0.15

            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"Distance to tag = {distance:.2f}"
            )

            return

        # FINISHED
        self.cmd_pub.publish(Twist())

        self.mission_complete = True

        self.get_logger().info(
            ">>>>>>>> TAG REACHED <<<<<<<<"
        )

    def map_is_ready(self):

        if self.map_data is None:
            return False
        
        known = np.count_nonzero(
            self.map_data != -1
        )

        total = self.map_data.size

        if total == 0:
            return False

        return (known / total) > 0.01


def main(args=None):

    rclpy.init(args=args)

    node = Explorer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()