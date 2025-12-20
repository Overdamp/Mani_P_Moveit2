#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from geometry_msgs.msg import PoseStamped, Quaternion  # นำเข้า message types
from moveit_msgs.msg import CollisionObject  # นำเข้า message types สำหรับวัตถุชน
from shape_msgs.msg import SolidPrimitive  # นำเข้า message types สำหรับรูปทรงพื้นฐาน
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import math  # นำเข้า math
from collections import deque, defaultdict  # นำเข้า deque และ defaultdict

class CubeSpawner(Node):

    def __init__(self):
        super().__init__('cube_spawner')  # สร้าง Node ชื่อ 'cube_spawner'

        # ==========================================
        # 🛠️ CONFIG ZONE 🛠️ (ส่วนตั้งค่า)
        # ==========================================
        self.target_tags = ["tag1", "tag2", "tag3"]  # รายชื่อ Tag ที่ต้องการสร้าง Cube ทับ
        self.base_frame = "Base_link"  # Frame อ้างอิงหลัก
        self.cube_size = 0.05  # ขนาดของ Cube (เมตร)
        
        # Smoothing Config (การตั้งค่าความนุ่มนวล)
        self.buffer_size = 5  # จำนวน Frame ที่นำมาเฉลี่ย (Moving Average)
        self.timer_period = 0.1 # ความถี่ในการอัปเดต (10Hz)
        # ==========================================

        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)  # Publisher สำหรับส่ง Collision Object
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data Storage for Smoothing (ที่เก็บข้อมูลสำหรับคำนวณค่าเฉลี่ย)
        # Key: tag_id, Value: dict of deques (เก็บ x, y, z, sin_yaw, cos_yaw)
        self.buffers = defaultdict(lambda: {
            'x': deque(maxlen=self.buffer_size),
            'y': deque(maxlen=self.buffer_size),
            'z': deque(maxlen=self.buffer_size),
            'sin_yaw': deque(maxlen=self.buffer_size),
            'cos_yaw': deque(maxlen=self.buffer_size)
        })

        self.timer = self.create_timer(self.timer_period, self.update_objects)  # เริ่ม Timer
        self.get_logger().info(f'Cube Spawner (Smoothed) Started for: {self.target_tags}')

    def get_euler_from_quaternion(self, q):
        # แปลง Quaternion เป็น Euler
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        # แปลง Euler เป็น Quaternion
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def update_objects(self):
        # ฟังก์ชันหลักที่ทำงานวนลูปเพื่ออัปเดตตำแหน่ง Cube
        for tag_id in self.target_tags:
            try:
                # 1. ดึงค่า Transform ดิบจาก TF
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))

                raw_x = transform.transform.translation.x
                raw_y = transform.transform.translation.y
                raw_z = transform.transform.translation.z
                _, _, raw_yaw = self.get_euler_from_quaternion(transform.transform.rotation)

                # 2. เพิ่มค่าลงใน Buffer
                buf = self.buffers[tag_id]
                buf['x'].append(raw_x)
                buf['y'].append(raw_y)
                buf['z'].append(raw_z)
                buf['sin_yaw'].append(math.sin(raw_yaw))
                buf['cos_yaw'].append(math.cos(raw_yaw))

                if len(buf['x']) < 1: continue

                # 3. คำนวณค่าเฉลี่ย (Average)
                avg_x = sum(buf['x']) / len(buf['x'])
                avg_y = sum(buf['y']) / len(buf['y'])
                avg_z = sum(buf['z']) / len(buf['z'])
                
                avg_sin = sum(buf['sin_yaw']) / len(buf['sin_yaw'])
                avg_cos = sum(buf['cos_yaw']) / len(buf['cos_yaw'])
                avg_yaw = math.atan2(avg_sin, avg_cos)

                # 4. สร้าง Collision Object (บังคับ Roll=0, Pitch=0)
                obj = CollisionObject()
                obj.header.frame_id = self.base_frame
                obj.id = f"cube_{tag_id}" 

                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [self.cube_size, self.cube_size, self.cube_size]

                pose = PoseStamped()
                pose.pose.position.x = avg_x
                pose.pose.position.y = avg_y
                pose.pose.position.z = avg_z 
                
                # บังคับให้ Roll และ Pitch เป็น 0 เพื่อให้ Cube ตั้งตรงเสมอ
                q_list = self.get_quaternion_from_euler(0.0, 0.0, avg_yaw)
                pose.pose.orientation = Quaternion(x=q_list[0], y=q_list[1], z=q_list[2], w=q_list[3])

                obj.primitives.append(primitive)
                obj.primitive_poses.append(pose.pose)
                obj.operation = CollisionObject.ADD  # สั่งเพิ่มวัตถุ

                self.collision_pub.publish(obj)

            except Exception:
                continue # ถ้าหา TF ไม่เจอก็ข้ามไป

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    node = CubeSpawner()  # สร้าง Node
    rclpy.spin(node)  # หมุน loop
    node.destroy_node()
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()