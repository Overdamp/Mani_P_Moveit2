#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from apriltag_msgs.msg import AprilTagDetectionArray  # นำเข้า message types สำหรับ AprilTag

class TagChecker(Node):
    def __init__(self):
        super().__init__('tag_checker')  # สร้าง Node ชื่อ 'tag_checker'
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',  # หัวข้อมาตรฐานสำหรับ apriltag_ros
            self.listener_callback,
            10)
        self.get_logger().info('👀 Waiting for AprilTags on /detections ...')

    def listener_callback(self, msg):
        # Callback เมื่อได้รับข้อมูล AprilTag
        if not msg.detections:
            # self.get_logger().info('No tags detected.')
            pass
        else:
            ids = [d.id for d in msg.detections]  # ดึง ID ของ Tag ที่เจอ
            self.get_logger().info(f'✅ Detected Tags: {ids}')

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    node = TagChecker()  # สร้าง Node
    rclpy.spin(node)  # หมุน loop
    node.destroy_node()
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
