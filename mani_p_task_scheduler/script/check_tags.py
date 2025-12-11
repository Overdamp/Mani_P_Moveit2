#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray

class TagChecker(Node):
    def __init__(self):
        super().__init__('tag_checker')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',  # Standard topic for apriltag_ros
            self.listener_callback,
            10)
        self.get_logger().info('👀 Waiting for AprilTags on /detections ...')

    def listener_callback(self, msg):
        if not msg.detections:
            # self.get_logger().info('No tags detected.')
            pass
        else:
            ids = [d.id for d in msg.detections]
            self.get_logger().info(f'✅ Detected Tags: {ids}')

def main(args=None):
    rclpy.init(args=args)
    node = TagChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
