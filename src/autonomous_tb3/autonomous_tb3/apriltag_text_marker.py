import rclpy

from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray

from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point



class AprilTagText(Node):

    def __init__(self):

        super().__init__('apriltag_text')

        self.sub = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/apriltag_text_marker',
            10
        )

    def detection_callback(self, msg):

        if len(msg.detections) == 0:
            return

        detection = msg.detections[0]

        tag_id = detection.id

        marker = Marker()

        marker.header.frame_id = "camera_link"

        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "apriltag_text"

        marker.id = 0

        marker.type = Marker.TEXT_VIEW_FACING

        marker.action = Marker.ADD

        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.5

        marker.pose.orientation.w = 1.0

        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.text = f"TAG ID: {tag_id}"

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = AprilTagText()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()