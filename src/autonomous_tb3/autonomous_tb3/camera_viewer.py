#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_viewer')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.get_logger().info("Camera viewer started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow("Robot Camera", frame)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = CameraViewer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()