#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

class CubeSpawner(Node):

    def __init__(self):
        super().__init__('cube_spawner')

        # ==========================================
        # 🛠️ CONFIG ZONE 🛠️
        # ==========================================
        
        # 1. รายชื่อ Tag ที่ต้องการให้เป็น "ลูกบาศก์"
        # (เช็คชื่อ Frame จริงใน tf_echo ด้วยนะครับ บางทีอาจเป็น tag36h11_1)
        self.target_tags = ["tag1", "tag2", "tag3"]
        
        # 2. เฟรมอ้างอิง
        self.base_frame = "Base_link" # เช็คตัวพิมพ์เล็ก/ใหญ่ ให้ตรงกับ URDF
        
        # 3. ขนาดวัตถุ (เมตร)
        # 50mm = 0.05m
        self.cube_size = 0.05 

        # ==========================================

        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ทำงานทุกๆ 0.5 วินาที (2Hz) ก็เพียงพอสำหรับวัตถุเป้าหมาย
        self.timer = self.create_timer(0.5, self.update_objects)
        self.get_logger().info(f'Cube Spawner Started for tags: {self.target_tags}')

    def update_objects(self):
        # วนลูปเช็ค Tag ทีละตัวใน list
        for tag_id in self.target_tags:
            try:
                # 1. หาตำแหน่ง Tag
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, 
                    tag_id, 
                    rclpy.time.Time(), 
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )

                # 2. สร้าง Collision Object
                obj = CollisionObject()
                obj.header.frame_id = self.base_frame
                
                # ตั้งชื่อ ID ให้ไม่ซ้ำกัน (เช่น cube_tag1, cube_tag2)
                obj.id = f"cube_{tag_id}" 

                # 3. กำหนดรูปร่าง (BOX 50mm)
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [self.cube_size, self.cube_size, self.cube_size]

                # 4. กำหนดตำแหน่ง
                pose = PoseStamped()
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation

                # [Optional] ปรับ Offset ถ้่า Tag แปะอยู่ "ผิวหน้า" ลูกบาศก์
                # เราอาจต้องขยับจุดกึ่งกลางถอยหลังไปครึ่งก้อน (0.025m)
                # pose.pose.position.z -= (self.cube_size / 2.0)

                obj.primitives.append(primitive)
                obj.primitive_poses.append(pose.pose)
                obj.operation = CollisionObject.ADD

                # 5. Publish
                self.collision_pub.publish(obj)
                # self.get_logger().info(f'Spawned {obj.id}')

            except Exception:
                # ถ้าไม่เจอ Tag นี้ ก็ข้ามไปเช็คตัวถัดไปเงียบๆ
                continue

def main(args=None):
    rclpy.init(args=args)
    node = CubeSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()