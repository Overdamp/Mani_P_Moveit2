#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import sys
import numpy as np

class FisheyeCameraNode(Node):
    def __init__(self):
        super().__init__('fisheye_camera_node')
        
        # Parameters
        self.declare_parameter('device_id', 1) # ค่าเริ่มต้น (ถ้าหาไม่เจอ เดี๋ยวระบบหาใหม่ให้)
        self.declare_parameter('frame_id', 'fisheye_camera_link')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        self.target_device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # QoS Settings (Best Effort เพื่อความลื่นไหล)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.image_pub = self.create_publisher(Image, 'fisheye_camera/image_raw', qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, 'fisheye_camera/camera_info', qos_profile)
        
        # --- Auto-Detect Camera Logic ---
        self.cap = self.find_and_open_camera(self.target_device_id)
        
        if self.cap is None:
            self.get_logger().error("FATAL: No working camera found on any index (0-10).")
            sys.exit(1)
            
        # Set Camera Properties (MJPG is crucial for USB bandwidth)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # Double check what we actually got
        actual_w = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_h = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera running on /dev/video{self.current_device_id} | Resolution: {actual_w}x{actual_h}")
        
        # Timer
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def find_and_open_camera(self, preferred_id):
        """
        ฟังก์ชันสำหรับลองเปิดกล้อง
        1. ลอง ID ที่ user ตั้งมาก่อน
        2. ถ้าไม่ได้ ให้ลองวนหา 0-10
        """
        # ลองเปิดตัวที่ตั้งค่ามา
        self.get_logger().info(f"Attempting to open preferred device: /dev/video{preferred_id}")
        cap = cv2.VideoCapture(preferred_id)
        if self.check_camera_works(cap):
            self.current_device_id = preferred_id
            return cap
        
        # ถ้าเปิดไม่ได้ ให้ปิดและลองหาตัวอื่น
        self.get_logger().warn(f"Preferred device {preferred_id} failed. Scanning other indices...")
        cap.release()
        
        for i in range(10):
            if i == preferred_id: continue # ข้ามตัวที่ลองไปแล้ว
            
            cap = cv2.VideoCapture(i)
            if self.check_camera_works(cap):
                self.get_logger().info(f"Found working camera at index: {i}")
                self.current_device_id = i
                return cap
            cap.release()
            
        return None

    def check_camera_works(self, cap):
        """เช็คว่าเปิดได้และอ่านภาพได้จริงไหม"""
        if not cap.isOpened():
            return False
        # ลองอ่าน 1 เฟรมเพื่อความชัวร์
        ret, frame = cap.read()
        return ret

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            stamp = self.get_clock().now().to_msg()
            
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            
            self.image_pub.publish(msg)
            
            # Dummy Camera Info
            info_msg = CameraInfo()
            info_msg.header.stamp = stamp
            info_msg.header.frame_id = self.frame_id
            info_msg.width = self.width
            info_msg.height = self.height
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = [0.0]*5
            info_msg.k = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info_msg.p = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]
            
            self.info_pub.publish(info_msg)
        else:
            self.get_logger().warn("Failed to capture frame")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
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