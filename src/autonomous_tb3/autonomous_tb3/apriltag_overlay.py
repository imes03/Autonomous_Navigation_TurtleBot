import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from apriltag_msgs.msg import AprilTagDetectionArray

from cv_bridge import CvBridge

import cv2



class AprilTagOverlay(Node):

    def __init__(self):

        super().__init__('apriltag_overlay')

        self.bridge = CvBridge()

        self.latest_detections = []

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.image_pub = self.create_publisher(
            Image,
            '/camera/tag_overlay',
            10
        )

    def detection_callback(self, msg):

        self.latest_detections = msg.detections

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        for detection in self.latest_detections:

            tag_id = detection.id

            cx = int(detection.centre.x)

            cy = int(detection.centre.y)

            cv2.putText(
                frame,
                f"ID: {tag_id}",
                (cx, cy),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2
            )

            cv2.circle(
                frame,
                (cx, cy),
                5,
                (0, 255, 0),
                -1
            )

        out_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.image_pub.publish(out_msg)


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagOverlay()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()