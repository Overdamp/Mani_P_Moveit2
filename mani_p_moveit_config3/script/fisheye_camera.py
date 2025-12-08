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
        
        # ==========================================
        # 1. SETTINGS & PARAMETERS
        # ==========================================
        self.declare_parameter('device_id', 1)      # เริ่มหาจาก video1
        self.declare_parameter('frame_id', 'fisheye_camera_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # ตัวแปรสำหรับเปิด/ปิด การแก้ภาพโค้ง (Undistort)
        # False = โหมด Calibrate (ภาพดิบ แต่หมุนแล้ว) -> ใช้ตอนทำ Calibration
        # True  = โหมดใช้งานจริง (ภาพตรง) -> ใช้ตอนรัน AprilTag
        self.RECTIFY_MODE = False 

        # ==========================================
        # 2. CALIBRATION DATA (ใส่ค่าที่ได้จากการ Calibrate ตรงนี้)
        # ==========================================
        # ค่าสมมติ (ต้องเปลี่ยนเป็นเลขจริงที่คุณได้จาก ROS Calibration)
        self.K = np.array([
            [400.0, 0.0, 320.0],
            [0.0, 400.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        self.D = np.array([-0.1, 0.05, 0.0, 0.0, 0.0]) # k1, k2, p1, p2, k3

        # รับค่า Parameter
        self.target_device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        # คำนวณ Optimal Matrix สำหรับการแก้ภาพ (ใช้เมื่อ RECTIFY_MODE = True)
        if self.RECTIFY_MODE:
            self.new_K, self.roi = cv2.getOptimalNewCameraMatrix(
                self.K, self.D, (self.width, self.height), 1, (self.width, self.height)
            )

        # ==========================================
        # 3. SETUP ROS & CAMERA
        # ==========================================
        # QoS Profile: Best Effort เพื่อความลื่นไหลของวิดีโอ
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Topic Name จะเปลี่ยนตามโหมด
        topic_name = 'fisheye_camera/image_rect' if self.RECTIFY_MODE else 'fisheye_camera/image_raw'
        self.image_pub = self.create_publisher(Image, topic_name, qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, 'fisheye_camera/camera_info', qos_profile)
        
        # Auto-Detect Camera
        self.cap = self.find_and_open_camera(self.target_device_id)
        
        if self.cap is None:
            self.get_logger().error("FATAL: No working camera found.")
            sys.exit(1)
            
        # Force MJPG (สำคัญมากสำหรับ USB Bandwidth)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.get_logger().info(f"Camera ready on /dev/video{self.current_device_id} (Rotated 180)")

        # Timer
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def find_and_open_camera(self, preferred_id):
        """วนหา Device ID อัตโนมัติ"""
        # ลองตัวที่ตั้งค่ามาก่อน
        cap = cv2.VideoCapture(preferred_id)
        if self.check_camera(cap):
            self.current_device_id = preferred_id
            return cap
        
        # ถ้าไม่ได้ ให้วนหา 0-10
        self.get_logger().warn(f"Device {preferred_id} failed. Scanning others...")
        cap.release()
        for i in range(10):
            if i == preferred_id: continue
            cap = cv2.VideoCapture(i)
            if self.check_camera(cap):
                self.get_logger().info(f"Found camera at index {i}")
                self.current_device_id = i
                return cap
            cap.release()
        return None

    def check_camera(self, cap):
        if not cap.isOpened(): return False
        ret, _ = cap.read()
        return ret

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # 1. หมุนภาพ 180 องศา (ทำเป็นอย่างแรกสุด)
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # 2. แก้ภาพโค้ง (ถ้าเปิดโหมด Rectify)
            if self.RECTIFY_MODE:
                frame = cv2.undistort(frame, self.K, self.D, None, self.new_K)
                # (Optional) Crop ภาพถ้าต้องการตัดขอบดำ
                # x, y, w, h = self.roi
                # frame = frame[y:y+h, x:x+w]

            # 3. แปลงเป็น ROS Message
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height = frame.shape[0]
            msg.width = frame.shape[1]
            msg.encoding = "bgr8"
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            
            self.image_pub.publish(msg)
            
            # 4. Publish Camera Info (ส่งค่า K, D ไปด้วยเผื่อ node อื่นใช้)
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = self.width
            info_msg.height = self.height
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = self.D.tolist()
            info_msg.k = self.K.flatten().tolist()
            info_msg.p = [self.K[0,0], 0., self.K[0,2], 0., 
                          0., self.K[1,1], self.K[1,2], 0., 
                          0., 0., 1., 0.]
            self.info_pub.publish(info_msg)

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