#!/usr/bin/env python3
# ระบุว่าสคริปต์นี้รันด้วย python3

import rclpy
# นำเข้าไลบรารี rclpy สำหรับการเขียน ROS 2 Node
from rclpy.node import Node
# นำเข้าคลาส Node เพื่อสร้าง ROS 2 Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
# นำเข้าเครื่องมือสำหรับจัดการ TF (Buffer, Listener) และส่ง TF (Broadcaster)
from geometry_msgs.msg import TransformStamped, Quaternion
# นำเข้า message types สำหรับ TF
import math
# นำเข้าไลบรารีคณิตศาสตร์
import tf_transformations
# นำเข้าไลบรารีสำหรับการแปลงค่าทางคณิตศาสตร์ของ TF (เช่น Quaternion <-> Euler)

class TagLeveler(Node):
    # สร้างคลาส TagLeveler โดยสืบทอดมาจาก Node

    def __init__(self):
        # ฟังก์ชันเริ่มต้น (Constructor) ของคลาส
        super().__init__('tag_leveler')
        # เรียกใช้ Constructor ของคลาสแม่ (Node) และตั้งชื่อ Node ว่า 'tag_leveler'

        # ==========================================
        # 🛠️ ตั้งค่า (CONFIG) 🛠️
        # ==========================================
        self.target_tags = ["tag1", "tag2", "tag3", "tag4", "tag5", "tag6"]
        # รายชื่อ Tag ที่ต้องการปรับระนาบ (เพิ่มลดได้ตามต้องการ)
        self.base_frame = "Base_link"
        # เฟรมอ้างอิงหลัก (World Frame)
        self.suffix = "_level"
        # คำต่อท้ายชื่อ Frame ใหม่ (เช่น tag1 -> tag1_level)
        self.timer_period = 0.05 
        # ความถี่ในการทำงาน (0.05 วินาที = 20Hz)
        # ==========================================

        self.tf_buffer = Buffer()
        # สร้าง Buffer สำหรับเก็บข้อมูล TF
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # สร้าง Listener เพื่อรอรับข้อมูล TF
        self.tf_broadcaster = TransformBroadcaster(self)
        # สร้าง Broadcaster เพื่อส่งข้อมูล TF ใหม่

        self.timer = self.create_timer(self.timer_period, self.update_transforms)
        # สร้าง Timer เพื่อเรียกฟังก์ชัน update_transforms ตามเวลาที่กำหนด
        
        self.get_logger().info(f'Tag Leveler Started. Publishing *{self.suffix} frames.')
        # แสดงข้อความแจ้งเตือนว่า Node เริ่มทำงานแล้ว

    def get_euler_from_quaternion(self, q):
        # ฟังก์ชันแปลง Quaternion เป็น Euler Angles (Roll, Pitch, Yaw)
        # ใช้ tf_transformations เพื่อความแม่นยำและง่าย
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return roll, pitch, yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        # ฟังก์ชันแปลง Euler Angles เป็น Quaternion
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def update_transforms(self):
        # ฟังก์ชันหลักที่ทำงานวนลูปเพื่ออัปเดต TF
        
        for tag_id in self.target_tags:
            # วนลูปตามรายชื่อ Tag ที่ต้องการ
            try:
                # 1. ดึงข้อมูล Transform ดิบจาก TF (จาก Base_link ไปยัง Tag)
                # ใช้ Timeout น้อยๆ เพื่อไม่ให้บล็อคการทำงานนานเกินไป
                if not self.tf_buffer.can_transform(self.base_frame, tag_id, rclpy.time.Time()):
                    continue
                    
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_id, rclpy.time.Time())

                # 2. ดึงค่าตำแหน่ง (Position) - ใช้ค่าเดิม
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z

                # 3. ดึงค่าการหมุน (Orientation) และแปลงเป็น Euler
                rx, ry, rz = self.get_euler_from_quaternion(t.transform.rotation)

                # 4. สร้าง Quaternion ใหม่โดยบังคับ Roll=0, Pitch=0
                # ใช้ค่า Yaw (rz) เดิม เพื่อให้ทิศทางหันหน้ายังคงเดิม แต่ระนาบไปกับพื้น
                q_new = self.get_quaternion_from_euler(0.0, 0.0, rz)

                # 5. สร้าง TransformStamped message ใหม่
                t_new = TransformStamped()
                t_new.header.stamp = self.get_clock().now().to_msg()
                # ใช้วเวลาปัจจุบัน
                t_new.header.frame_id = self.base_frame
                # เฟรมแม่คือ Base_link
                t_new.child_frame_id = f"{tag_id}{self.suffix}"
                # เฟรมลูกคือชื่อเดิม + suffix (เช่น tag1_level)

                t_new.transform.translation.x = tx
                t_new.transform.translation.y = ty
                t_new.transform.translation.z = tz
                # ตำแหน่งเหมือนเดิม

                t_new.transform.rotation = q_new
                # การหมุนแบบปรับระนาบแล้ว

                # 6. ส่งค่า TF ใหม่ออกไป
                self.tf_broadcaster.sendTransform(t_new)

            except Exception as e:
                # ถ้าเกิดข้อผิดพลาด (เช่น หา TF ไม่เจอ) ให้ข้ามไปเงียบๆ
                pass

def main(args=None):
    # ฟังก์ชันหลัก
    rclpy.init(args=args)
    node = TagLeveler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
