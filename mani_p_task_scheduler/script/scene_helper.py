#!/usr/bin/env python3
import rclpy
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class SceneHelper:
    def __init__(self, node):
        self.node = node
        # Publisher สำหรับส่งคำสั่งเข้า Planning Scene
        self.scene_pub = self.node.create_publisher(PlanningScene, '/planning_scene', 10)

    def attach_box(self, object_id, link_name, touch_links, size=[0.05, 0.05, 0.05]):
        """
        แนบวัตถุ (Box) เข้ากับหุ่นยนต์ (Attach)
        object_id: ชื่อ ID ของวัตถุ (เช่น 'cube1')
        link_name: ชื่อ Link ที่วัตถุจะไปติด (เช่น 'tcp_link' หรือ 'palm_link')
        touch_links: รายชื่อ Link ที่อนุญาตให้ชนกับวัตถุได้ (คือนิ้วมือนั่นเอง)
        size: ขนาดของกล่อง [x, y, z]
        """
        self.node.get_logger().info(f"🔗 Attaching {object_id} to {link_name}")

        attached_object = AttachedCollisionObject()
        attached_object.link_name = link_name
        attached_object.object.header.frame_id = link_name
        attached_object.object.id = object_id
        attached_object.object.operation = CollisionObject.ADD
        
        # กำหนดขนาดวัตถุ
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        attached_object.object.primitives.append(primitive)
        
        # กำหนด Pose (เทียบกับ link_name)
        # ถือว่ามันอยู่ตรงกลางมือ (0,0,0)
        pose = Pose()
        pose.orientation.w = 1.0
        attached_object.object.primitive_poses.append(pose)

        # *** สำคัญมาก: Touch Links ***
        # บอก MoveIt ว่า "อย่าฟ้องว่าชนนะ ถ้านิ้วพวกนี้โดนวัตถุ"
        attached_object.touch_links = touch_links 

        # สร้าง Planning Scene Message
        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(attached_object)
        
        # ลบออกจาก World (Collision Object ธรรมดา) เพื่อไม่ให้ซ้ำซ้อน
        remove_object = CollisionObject()
        remove_object.id = object_id
        remove_object.operation = CollisionObject.REMOVE
        scene_msg.world.collision_objects.append(remove_object)

        self.scene_pub.publish(scene_msg)

    def detach_box(self, object_id, frame_id="Base_link", drop_pose=None, size=[0.05, 0.05, 0.05]):
        """
        ปล่อยวัตถุออกจากมือ กลับสู่โลก (Detach)
        """
        self.node.get_logger().info(f"🔓 Detaching {object_id}")

        # 1. ลบออกจาก Attached Object
        detach_object = AttachedCollisionObject()
        detach_object.object.id = object_id
        detach_object.object.operation = CollisionObject.REMOVE
        
        # 2. เพิ่มกลับเข้าไปใน World (Collision Object)
        add_object = CollisionObject()
        add_object.id = object_id
        add_object.header.frame_id = frame_id
        add_object.operation = CollisionObject.ADD
        
        # กำหนดรูปร่างอีกครั้ง
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size
        add_object.primitives.append(primitive)
        
        # กำหนดตำแหน่งที่จะวาง
        if drop_pose:
            add_object.primitive_poses.append(drop_pose)
        else:
            # ถ้าไม่มี drop_pose ให้ใช้ตำแหน่งปัจจุบัน (อาจต้องใช้ TF ช่วยหาตำแหน่งจริง แต่เบื้องต้นใส่ 0,0,0 ไปก่อน หรือควรส่ง drop_pose มา)
            # เพื่อความปลอดภัย ควรส่ง drop_pose มาเสมอ
            self.node.get_logger().warn("⚠️ No drop_pose provided for detach. Object might appear at origin.")
            default_pose = Pose()
            default_pose.orientation.w = 1.0
            add_object.primitive_poses.append(default_pose)

        scene_msg = PlanningScene()
        scene_msg.is_diff = True
        scene_msg.robot_state.is_diff = True
        scene_msg.robot_state.attached_collision_objects.append(detach_object)
        scene_msg.world.collision_objects.append(add_object)

        self.scene_pub.publish(scene_msg)
