#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import sys
import numpy as np
import yaml

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class FisheyeCameraNode(Node):
    def __init__(self):
        super().__init__('fisheye_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 3)
        self.declare_parameter('frame_id', 'fisheye_camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('rotation', 0) # 0, 90, 180, 270
        
        self.device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.calib_file = self.get_parameter('calibration_file').value
        self.rotation = self.get_parameter('rotation').value
        
        # QoS Profile (Reliable)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'image_raw', qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, 'camera_info', qos_profile)
        
        # Bridge
        self.bridge = CvBridge()
        
        # Load Calibration
        self.camera_info = self.load_camera_info()
        
        # Open Camera
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
        
        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open video device {self.device_id}")
            sys.exit(1)
            
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.get_logger().info(f"Fisheye Camera Node Started on /dev/video{self.device_id}")
        
        # Timer
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def load_camera_info(self):
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        
        if self.calib_file:
            try:
                with open(self.calib_file, 'r') as f:
                    calib = yaml.safe_load(f)
                    info.distortion_model = calib.get('distortion_model', 'plumb_bob')
                    info.d = calib.get('distortion_coefficients', {}).get('data', [])
                    info.k = calib.get('camera_matrix', {}).get('data', [])
                    info.r = calib.get('rectification_matrix', {}).get('data', [])
                    info.p = calib.get('projection_matrix', {}).get('data', [])
                    self.get_logger().info(f"Loaded calibration from {self.calib_file}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load calibration: {e}")
        
        # Defaults if empty or zero
        if len(info.k) == 0 or all(x == 0 for x in info.k):
            info.distortion_model = "plumb_bob"
            info.d = [0.0]*5
            info.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            
        return info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to read frame")
            return
        
        # Rotation
        if self.rotation == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            
        # Update Dimensions if rotated
        h, w = frame.shape[:2]
        
        stamp = self.get_clock().now().to_msg()
        
        # Image Msg (Using CvBridge)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        self.image_pub.publish(msg)
        self.get_logger().info(f"Published Image (Subs: {self.image_pub.get_subscription_count()})") # Debug
        
        # Camera Info Msg
        self.camera_info.header.stamp = stamp
        self.camera_info.width = w
        self.camera_info.height = h
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FisheyeCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
