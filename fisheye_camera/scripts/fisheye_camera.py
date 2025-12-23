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
        self.declare_parameter('device_id', '3') # Changed to string to support paths like /dev/video0
        self.declare_parameter('frame_id', 'fisheye_camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 5) # Reduced to 5 for stability
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('rotation', 0) # 0, 90, 180, 270
        
        self.device_id_param = self.get_parameter('device_id').value
        self.device_id = self.resolve_device_id(self.device_id_param)
        self.get_logger().info(f"Resolved Device ID: {self.device_id} (from {self.device_id_param})")
            
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # Override FPS to 5 if it's failing
        # self.fps = 5 
        
        self.calib_file = self.get_parameter('calibration_file').value
        self.rotation = self.get_parameter('rotation').value

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
            
        # Force MJPG to save USB bandwidth
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.get_logger().info(f"Fisheye Camera Node Started on {self.device_id}")
        
        # Timer
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def resolve_device_id(self, device_arg):
        """
        Resolves the device argument to an integer index for OpenCV.
        Handles: '0', '/dev/video0', '/dev/fisheye_camera' (symlink)
        """
        import os
        import re
        
        # 1. If it's already a digit, return int
        if str(device_arg).isdigit():
            return int(device_arg)
            
        # 2. If it's a path, resolve symlink
        if os.path.exists(device_arg):
            real_path = os.path.realpath(device_arg) # e.g. /dev/video2
            # Extract number
            match = re.search(r'video(\d+)', real_path)
            if match:
                return int(match.group(1))
                
        self.get_logger().warn(f"Could not resolve device {device_arg} to index. Using default 0.")
        return 0
        
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
            
        # Force MJPG to save USB bandwidth
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.get_logger().info(f"Fisheye Camera Node Started on {self.device_id}")
        
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
        
        # Add waitKey to help OpenCV event processing
        cv2.waitKey(1)
        
        if not ret:
            self.get_logger().warn("Failed to read frame. Attempting to reconnect...")
            self.cap.release()
            # Try to reopen
            self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            if not self.cap.isOpened():
                 self.get_logger().error("Reconnect failed.")
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
        
        # Undistort Image - REMOVED (Let image_proc handle it)
        # if self.camera_info.d:
        #     K = np.array(self.camera_info.k).reshape(3, 3)
        #     D = np.array(self.camera_info.d)
        #     P = np.array(self.camera_info.p).reshape(3, 4)
        #     new_K = P[:3, :3]
        #     frame_rect = cv2.undistort(frame, K, D, None, new_K)
        # else:
        #     frame_rect = frame
        
        frame_rect = frame # Publish Raw

        stamp = self.get_clock().now().to_msg()
        
        # Image Msg (Raw)
        msg = self.bridge.cv2_to_imgmsg(frame_rect, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        self.image_pub.publish(msg)
        
        if not hasattr(self, 'logged_once'):
            self.get_logger().info(f"Published Raw Image (Subs: {self.image_pub.get_subscription_count()})")
            self.logged_once = True
        
        # Camera Info Msg (Raw Model)
        self.camera_info.header.stamp = stamp
        self.camera_info.width = w
        self.camera_info.height = h
        # Do NOT zero out D
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
