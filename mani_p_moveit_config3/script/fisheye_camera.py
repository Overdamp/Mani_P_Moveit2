#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from sensor_msgs.msg import Image, CameraInfo  # นำเข้า message types สำหรับภาพและข้อมูลกล้อง
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy  # นำเข้า QoS settings
import cv2  # นำเข้า OpenCV
import sys  # นำเข้า sys
import numpy as np  # นำเข้า numpy

class FisheyeCameraNode(Node):
    def __init__(self):
        super().__init__('fisheye_camera_node')  # สร้าง Node ชื่อ 'fisheye_camera_node'
        
        # ==========================================
        # 1. SETTINGS & PARAMETERS (การตั้งค่าและพารามิเตอร์)
        # ==========================================
        self.declare_parameter('device_id', 1)  # ประกาศ parameter device_id (default=1)
        self.declare_parameter('frame_id', 'fisheye_camera_optical_frame')  # ประกาศ parameter frame_id
        self.declare_parameter('width', 640)  # ความกว้างภาพ
        self.declare_parameter('height', 480)  # ความสูงภาพ
        self.declare_parameter('fps', 30)  # เฟรมเรต
        
        # ตั้งเป็น True เพื่อเริ่มแก้ภาพให้ตรง (Undistort)
        self.RECTIFY_MODE = True 

        # ==========================================
        # 2. CALIBRATION DATA (ค่าจริงของคุณ)
        # ==========================================
        # Camera Matrix (K) จาก Log ของคุณ
        self.K = np.array([
            [326.898006, 0.0, 311.476926],  # fx, 0, cx
            [0.0, 327.975714, 231.136870],  # 0, fy, cy
            [0.0, 0.0, 1.0]
        ])
        
        # Distortion Coeffs (D) จาก Log ของคุณ
        self.D = np.array([-0.285465, 0.067067, 0.000820, 0.002671, 0.0]) 

        # รับค่า Parameter ที่ประกาศไว้
        self.target_device_id = self.get_parameter('device_id').value
        self.frame_id = self.get_parameter('frame_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        # คำนวณ Optimal Matrix สำหรับการแก้ภาพ (Undistort)
        if self.RECTIFY_MODE:
            # alpha=1 หมายถึงเก็บ pixel ทั้งหมดไว้ (อาจมีขอบดำ)
            # alpha=0 หมายถึงซูมเข้าไปให้ไม่เห็นขอบดำ
            self.new_K, self.roi = cv2.getOptimalNewCameraMatrix(
                self.K, self.D, (self.width, self.height), 1, (self.width, self.height)
            )

        # ==========================================
        # 3. SETUP ROS & CAMERA (ตั้งค่า ROS และกล้อง)
        # ==========================================
        # กำหนด QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # ชื่อ Topic: ถ้าแก้ภาพแล้วใช้ชื่อ image_rect ให้ AprilTag เอาไปใช้ได้เลย
        topic_name = 'fisheye_camera/image_rect' if self.RECTIFY_MODE else 'fisheye_camera/image_raw'
        self.image_pub = self.create_publisher(Image, topic_name, qos_profile)  # สร้าง Publisher สำหรับภาพ
        self.info_pub = self.create_publisher(CameraInfo, 'fisheye_camera/camera_info', qos_profile)  # สร้าง Publisher สำหรับข้อมูลกล้อง
        
        # Auto-Detect Camera (ค้นหาและเปิดกล้องอัตโนมัติ)
        self.cap = self.find_and_open_camera(self.target_device_id)
        
        if self.cap is None:
            self.get_logger().error("FATAL: No working camera found.")  # แจ้ง error ถ้าหากล้องไม่เจอ
            sys.exit(1)
            
        # ตั้งค่ากล้อง OpenCV
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        self.get_logger().info(f"Camera ready on /dev/video{self.current_device_id} (Rotated 180)")

        # สร้าง Timer สำหรับอ่านภาพตาม FPS
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)

    def find_and_open_camera(self, preferred_id):
        # ฟังก์ชันค้นหากล้อง
        cap = cv2.VideoCapture(preferred_id)
        if self.check_camera(cap):
            self.current_device_id = preferred_id
            return cap
        
        self.get_logger().warn(f"Device {preferred_id} failed. Scanning others...")
        cap.release()
        # ลองวนหา index อื่นๆ
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
        # เช็คว่ากล้องเปิดได้และอ่านภาพได้จริงหรือไม่
        if not cap.isOpened(): return False
        ret, _ = cap.read()
        return ret

    def timer_callback(self):
        # ฟังก์ชันที่ทำงานทุกๆ Timer tick
        ret, frame = self.cap.read()
        
        if ret:
            # 1. หมุนภาพ 180 องศา (เนื่องจากกล้องติดกลับหัว)
            frame = cv2.rotate(frame, cv2.ROTATE_180)

            # 2. แก้ภาพโค้ง (Undistort)
            if self.RECTIFY_MODE:
                frame = cv2.undistort(frame, self.K, self.D, None, self.new_K)
                
                # ถ้าต้องการตัดขอบดำออก ให้ Uncomment 2 บรรทัดข้างล่างนี้
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
            
            self.image_pub.publish(msg)  # ส่งภาพออกไป
            
            # 4. Publish Camera Info (ส่งค่าที่ Calibrate แล้วไปให้ Node อื่น)
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = self.width
            info_msg.height = self.height
            info_msg.distortion_model = "plumb_bob"
            info_msg.d = self.D.tolist()
            info_msg.k = self.K.flatten().tolist()
            # Projection Matrix (P) จาก Log
            info_msg.p = [237.989360, 0.0, 315.616170, 0.0,
                          0.0, 266.872962, 227.688419, 0.0,
                          0.0, 0.0, 1.0, 0.0]
            
            self.info_pub.publish(info_msg)  # ส่งข้อมูลกล้องออกไป

    def destroy_node(self):
        # ปิดกล้องเมื่อ Node ถูกทำลาย
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    node = FisheyeCameraNode()  # สร้าง Node
    try:
        rclpy.spin(node)  # หมุน loop รอ event
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()  # ทำลาย Node
        rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()