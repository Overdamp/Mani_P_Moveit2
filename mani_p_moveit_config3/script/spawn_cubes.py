#!/usr/bin/env python3
# ระบุว่าสคริปต์นี้รันด้วย python3

import rclpy
# นำเข้าไลบรารี rclpy สำหรับการเขียน ROS 2 Node
from rclpy.node import Node
# นำเข้าคลาส Node เพื่อสร้าง ROS 2 Node
from geometry_msgs.msg import PoseStamped, Quaternion
# นำเข้า message types สำหรับตำแหน่ง (PoseStamped) และการหมุน (Quaternion)
from moveit_msgs.msg import CollisionObject
# นำเข้า message type สำหรับวัตถุชน (CollisionObject) ของ MoveIt
from shape_msgs.msg import SolidPrimitive
# นำเข้า message type สำหรับรูปทรงเรขาคณิต (SolidPrimitive) เช่น กล่อง
from tf2_ros import Buffer, TransformListener
# นำเข้าเครื่องมือสำหรับจัดการ TF (Transform)
import math
# นำเข้าไลบรารีคณิตศาสตร์
from collections import deque, defaultdict
# นำเข้า deque (คิว) และ defaultdict (ดิกชันนารีที่มีค่าเริ่มต้น) สำหรับการเก็บข้อมูล

class CubeSpawner(Node):
    # สร้างคลาส CubeSpawner โดยสืบทอดมาจาก Node

    def __init__(self):
        # ฟังก์ชันเริ่มต้น (Constructor) ของคลาส
        super().__init__('cube_spawner')
        # เรียกใช้ Constructor ของคลาสแม่ (Node) และตั้งชื่อ Node ว่า 'cube_spawner'

        # ==========================================
        # 🛠️ โซนตั้งค่า (CONFIG ZONE) 🛠️
        # ==========================================
        self.target_tags = ["tag1", "tag2", "tag3"]
        # รายชื่อ Tag ID ที่ต้องการตรวจจับเพื่อสร้างลูกบาศก์
        self.base_frame = "Base_link"
        # เฟรมอ้างอิงหลักของหุ่นยนต์ (Base Frame)
        self.cube_size = 0.05 
        # ขนาดของลูกบาศก์ (เมตร)
        
        # การปรับแต่งตำแหน่ง (Offset)
        self.z_offset = -0.025 
        # ปรับความสูง Z (เมตร) เพื่อให้ลูกบาศก์วางบนพื้นพอดี (ปรับตามความหนาของ Tag หรือตำแหน่งติดตั้ง)

        # การตั้งค่า Soft Snap (การดูดเข้าหามุมฉาก)
        self.snap_deadzone_deg = 5.0  
        # ช่วงมุม (องศา) ที่จะถือว่าเป็น 0 หรือ 90 องศา (Deadzone)
        self.snap_transition_deg = 15.0 
        # ช่วงมุม (องศา) ที่จะเริ่มทำการเกลี่ยค่า (Transition)

        # การตั้งค่า Smoothing (การลดสัญญาณรบกวน)
        self.buffer_size = 5  
        # จำนวนข้อมูลที่จะนำมาหาค่าเฉลี่ย (Moving Average)
        self.timer_period = 0.1 
        # คาบเวลาของ Timer (วินาที) -> 0.1 วินาที คือ 10Hz
        # ==========================================

        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        # สร้าง Publisher เพื่อส่งข้อมูล CollisionObject ไปยังหัวข้อ '/collision_object'

        self.tf_buffer = Buffer()
        # สร้าง Buffer สำหรับเก็บข้อมูล TF
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # สร้าง Listener เพื่อรอรับข้อมูล TF และเก็บลง Buffer

        # ตัวแปรสำหรับเก็บข้อมูลเพื่อทำ Smoothing
        # Key: tag_id, Value: dictionary ของ deque (คิว)
        self.buffers = defaultdict(lambda: {
            'x': deque(maxlen=self.buffer_size),
            # คิวเก็บค่าตำแหน่ง X
            'y': deque(maxlen=self.buffer_size),
            # คิวเก็บค่าตำแหน่ง Y
            'z': deque(maxlen=self.buffer_size),
            # คิวเก็บค่าตำแหน่ง Z
            'sin_yaw': deque(maxlen=self.buffer_size),
            # คิวเก็บค่า sin ของมุม Yaw (เพื่อเลี่ยงปัญหา Gimbal Lock / มุมกระโดด)
            'cos_yaw': deque(maxlen=self.buffer_size)
            # คิวเก็บค่า cos ของมุม Yaw
        })

        self.timer = self.create_timer(self.timer_period, self.update_objects)
        # สร้าง Timer เพื่อเรียกฟังก์ชัน update_objects ตามเวลาที่กำหนด
        self.get_logger().info(f'Cube Spawner (Smoothed + Soft Snap) Started for: {self.target_tags}')
        # แสดงข้อความแจ้งเตือนว่า Node เริ่มทำงานแล้ว

    def get_euler_from_quaternion(self, q):
        # ฟังก์ชันแปลง Quaternion เป็น Euler Angles (Roll, Pitch, Yaw)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        # คำนวณเทอม sin(roll) * cos(pitch)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        # คำนวณเทอม cos(roll) * cos(pitch)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # คำนวณมุม Roll

        sinp = 2 * (q.w * q.y - q.z * q.x)
        # คำนวณเทอม sin(pitch)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        # คำนวณมุม Pitch (พร้อมกันค่าเกินขอบเขต)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        # คำนวณเทอม sin(yaw) * cos(pitch)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        # คำนวณเทอม cos(yaw) * cos(pitch)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        # คำนวณมุม Yaw

        return roll, pitch, yaw
        # คืนค่า Roll, Pitch, Yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        # ฟังก์ชันแปลง Euler Angles เป็น Quaternion
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        # คำนวณ qx
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        # คำนวณ qy
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        # คำนวณ qz
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        # คำนวณ qw
        return [qx, qy, qz, qw]
        # คืนค่าเป็น list [x, y, z, w]

    def apply_soft_snap(self, current_yaw):
        # ฟังก์ชัน Soft Snap: ปรับมุม Yaw ให้เข้าหา 0, 90, 180, -90 โดยอัตโนมัติถ้าใกล้เคียง
        deg = math.degrees(current_yaw)
        # แปลงมุมเรเดียนเป็นองศา
        
        # ทำให้มุมอยู่ในช่วง -180 ถึง 180 เพื่อให้ง่ายต่อการคำนวณ
        while deg > 180: deg -= 360
        while deg <= -180: deg += 360
        
        # หา "มุมเป้าหมาย" ที่ใกล้ที่สุด (0, 90, 180, -90)
        # หารด้วย 90 แล้วปัดเศษ เพื่อดูว่าเป็นกี่เท่าของ 90
        target_deg = round(deg / 90.0) * 90.0
        
        # คำนวณความแตกต่างระหว่างมุมปัจจุบันกับมุมเป้าหมาย
        diff = abs(deg - target_deg)
        
        # ถ้าความแตกต่างน้อยกว่า Deadzone ให้ล็อคเข้าหามุมเป้าหมายเลย
        if diff <= self.snap_deadzone_deg:
            return math.radians(target_deg)
            # คืนค่ามุมเป้าหมาย (แปลงกลับเป็นเรเดียน)
            
        # ถ้าความแตกต่างมากกว่า Transition Zone ให้ใช้มุมเดิม (ไม่ Snap)
        if diff >= self.snap_transition_deg:
            return current_yaw
            # คืนค่ามุมเดิม
            
        # ถ้าอยู่ระหว่าง Deadzone และ Transition ให้เกลี่ยค่า (Linear Interpolation)
        # คำนวณอัตราส่วน (0.0 ถึง 1.0)
        ratio = (diff - self.snap_deadzone_deg) / (self.snap_transition_deg - self.snap_deadzone_deg)
        
        # คำนวณมุมผสมระหว่างมุมเป้าหมายและมุมจริง
        # ยิ่ง ratio มาก (ไกลจาก deadzone) ยิ่งใกล้มุมจริง
        # ยิ่ง ratio น้อย (ใกล้ deadzone) ยิ่งใกล้มุมเป้าหมาย
        # แต่สูตรข้างบน: ratio 0 คือที่ขอบ deadzone -> ควรเริ่ม snap
        # ratio 1 คือที่ขอบ transition -> ไม่ snap (ใช้มุมจริง)
        
        # ดังนั้น:
        # มุมที่ปรับ = มุมเป้าหมาย + (ส่วนต่าง * ratio) * เครื่องหมายของส่วนต่าง
        # หรือคิดง่ายๆ: เราลดขนาดของ diff ลง
        
        sign = 1 if deg > target_deg else -1
        # ทิศทางของความแตกต่าง
        
        # มุมใหม่ = มุมเป้าหมาย + (diff * ratio) * sign
        # ถ้า ratio = 0 -> มุมใหม่ = มุมเป้าหมาย (Snap 100%)
        # ถ้า ratio = 1 -> มุมใหม่ = มุมเป้าหมาย + diff = มุมจริง (Snap 0%)
        new_deg = target_deg + (diff * ratio * sign)
        
        return math.radians(new_deg)
        # คืนค่ามุมที่ปรับแล้วเป็นเรเดียน

    def update_objects(self):
        # ฟังก์ชันหลักที่ทำงานวนลูปเพื่ออัปเดตตำแหน่งลูกบาศก์
        for tag_id in self.target_tags:
            # วนลูปตามรายชื่อ Tag ที่ต้องการหา
            try:
                # 1. ดึงข้อมูล Transform ดิบจาก TF
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
                # พยายามหา Transform จาก Base_link ไปยัง Tag ID นั้นๆ (รอสูงสุด 0.05 วินาที)

                raw_x = transform.transform.translation.x
                # ดึงค่าตำแหน่ง X
                raw_y = transform.transform.translation.y
                # ดึงค่าตำแหน่ง Y
                raw_z = transform.transform.translation.z
                # ดึงค่าตำแหน่ง Z
                _, _, raw_yaw = self.get_euler_from_quaternion(transform.transform.rotation)
                # แปลง Quaternion เป็น Euler และดึงเฉพาะค่า Yaw (มุมหมุนรอบแกน Z)

                # 2. เก็บข้อมูลลง Buffer เพื่อหาค่าเฉลี่ย
                buf = self.buffers[tag_id]
                # ดึง Buffer ของ Tag นี้มา
                buf['x'].append(raw_x)
                # เพิ่มค่า X ลงคิว
                buf['y'].append(raw_y)
                # เพิ่มค่า Y ลงคิว
                buf['z'].append(raw_z)
                # เพิ่มค่า Z ลงคิว
                buf['sin_yaw'].append(math.sin(raw_yaw))
                # เพิ่มค่า sin(yaw) ลงคิว
                buf['cos_yaw'].append(math.cos(raw_yaw))
                # เพิ่มค่า cos(yaw) ลงคิว

                if len(buf['x']) < 1: continue
                # ถ้ายังไม่มีข้อมูลใน Buffer ให้ข้ามไปก่อน

                # 3. คำนวณค่าเฉลี่ย (Average)
                avg_x = sum(buf['x']) / len(buf['x'])
                # หาค่าเฉลี่ย X
                avg_y = sum(buf['y']) / len(buf['y'])
                # หาค่าเฉลี่ย Y
                avg_z = sum(buf['z']) / len(buf['z'])
                # หาค่าเฉลี่ย Z
                
                avg_sin = sum(buf['sin_yaw']) / len(buf['sin_yaw'])
                # หาค่าเฉลี่ย sin
                avg_cos = sum(buf['cos_yaw']) / len(buf['cos_yaw'])
                # หาค่าเฉลี่ย cos
                avg_yaw_raw = math.atan2(avg_sin, avg_cos)
                # แปลง sin/cos เฉลี่ยกลับเป็นมุม Yaw (วิธีนี้ช่วยจัดการปัญหามุม 180/-180 ได้ดีกว่าเฉลี่ยตรงๆ)

                # 4. ใช้ Soft Snap กับมุม Yaw
                final_yaw = self.apply_soft_snap(avg_yaw_raw)
                # เรียกฟังก์ชัน Soft Snap เพื่อปรับมุมให้เข้าฉาก

                # 5. สร้างวัตถุ Collision Object
                obj = CollisionObject()
                # สร้างออบเจกต์ CollisionObject
                obj.header.frame_id = self.base_frame
                # กำหนดเฟรมอ้างอิง
                obj.id = f"cube_{tag_id}" 
                # กำหนด ID ของวัตถุ (เช่น cube_tag1)

                primitive = SolidPrimitive()
                # สร้างรูปทรงเรขาคณิตพื้นฐาน
                primitive.type = SolidPrimitive.BOX
                # กำหนดประเภทเป็นกล่อง (BOX)
                primitive.dimensions = [self.cube_size, self.cube_size, self.cube_size]
                # กำหนดขนาด [กว้าง, ยาว, สูง]

                pose = PoseStamped()
                # สร้างตัวแปรเก็บตำแหน่งและทิศทาง
                pose.pose.position.x = avg_x
                # กำหนดตำแหน่ง X (ค่าเฉลี่ย)
                pose.pose.position.y = avg_y
                # กำหนดตำแหน่ง Y (ค่าเฉลี่ย)
                pose.pose.position.z = avg_z + self.z_offset
                # กำหนดตำแหน่ง Z (ค่าเฉลี่ย + Offset เพื่อวางบนพื้น)
                
                # บังคับให้ Roll = 0 และ Pitch = 0 เพื่อให้ลูกบาศก์วางราบกับพื้นเสมอ
                # ใช้ Yaw ที่ผ่านการ Soft Snap แล้ว
                q_list = self.get_quaternion_from_euler(0.0, 0.0, final_yaw)
                # แปลง Euler กลับเป็น Quaternion
                pose.pose.orientation = Quaternion(x=q_list[0], y=q_list[1], z=q_list[2], w=q_list[3])
                # กำหนดค่าการหมุนให้กับ Pose

                obj.primitives.append(primitive)
                # เพิ่มรูปทรงลงใน CollisionObject
                obj.primitive_poses.append(pose.pose)
                # เพิ่มตำแหน่งลงใน CollisionObject
                obj.operation = CollisionObject.ADD
                # กำหนดการกระทำเป็น ADD (เพิ่มวัตถุ)

                self.collision_pub.publish(obj)
                # ส่งข้อมูลออกไป (Publish)

            except Exception:
                # ถ้าเกิดข้อผิดพลาด (เช่น หา TF ไม่เจอ) ให้ข้ามไป
                continue

def main(args=None):
    # ฟังก์ชันหลัก (Main Function)
    rclpy.init(args=args)
    # เริ่มต้นระบบ ROS 2
    node = CubeSpawner()
    # สร้างอินสแตนซ์ของ CubeSpawner
    rclpy.spin(node)
    # วนลูปทำงาน Node ไปเรื่อยๆ
    node.destroy_node()
    # ทำลาย Node เมื่อหยุดทำงาน
    rclpy.shutdown()
    # ปิดระบบ ROS 2

if __name__ == '__main__':
    # ตรวจสอบว่าถูกเรียกใช้งานโดยตรงหรือไม่
    main()
    # เรียกฟังก์ชัน main
