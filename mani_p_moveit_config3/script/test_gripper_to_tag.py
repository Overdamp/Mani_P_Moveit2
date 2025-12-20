#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from moveit_msgs.action import MoveGroup  # นำเข้า action definition
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume  # นำเข้า message types สำหรับข้อจำกัด
from shape_msgs.msg import SolidPrimitive  # นำเข้า message types สำหรับรูปทรง
from geometry_msgs.msg import PoseStamped, Point, Quaternion  # นำเข้า message types
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys
import math  # นำเข้า math
import copy  # นำเข้า copy

class GripperToTag(Node):

    def __init__(self):
        super().__init__('gripper_to_tag')  # สร้าง Node ชื่อ 'gripper_to_tag'
        
        self.action_client = ActionClient(self, MoveGroup, 'move_action')  # สร้าง Action Client
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Config (การตั้งค่า)
        self.group_name = "arm"  # ชื่อกลุ่มแขนกล
        self.end_effector_link = "tcp_link"  # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"  # ชื่อ frame อ้างอิง
        self.approach_distance = 0.15 # ระยะห่าง 15cm จาก Tag

    def get_tag_transform(self, tag_name):
        # ฟังก์ชันดึงค่า Transform ของ Tag
        try:
            # รอ Transform
            self.get_logger().info(f"Waiting for transform {self.base_frame} -> {tag_name}...")
            # พยายามค้นหา Transform โดยมี Timeout 5 วินาที
            if not self.tf_buffer.can_transform(self.base_frame, tag_name, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)):
                self.get_logger().error(f"Could not find transform for {tag_name}")
                return None
            
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                tag_name,
                rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().error(f"TF Lookup Error: {e}")
            return None

    def send_goal(self, tag_name):
        # ฟังก์ชันส่ง Goal ไปยัง MoveGroup
        transform = self.get_tag_transform(tag_name)
        if transform is None:
            return

        goal_msg = MoveGroup.Goal()
        # กำหนด Workspace
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        goal_msg.request.start_state.is_diff = True
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.num_planning_attempts = 10

        # สร้าง Constraints (ข้อจำกัด)
        c = Constraints()
        c.name = "tag_approach"

        # 1. Position Constraint (ข้อจำกัดด้านตำแหน่ง)
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.end_effector_link
        pc.weight = 1.0
        
        # ตำแหน่งเป้าหมาย: Tag Position + Offset
        # สมมติว่า Tag Z ชี้ออกมาจากพื้นผิว
        # เราต้องการอยู่ข้างหน้ามัน
        # วิธีง่ายๆ: ใช้ตำแหน่ง Tag ไปก่อน แต่ในโค้ดจริงอาจต้องถอยออกมา
        # ในสคริปต์ทดสอบนี้ เราจะเล็งไปที่ตำแหน่ง Tag โดยตรง
        
        target_point = Point()
        target_point.x = transform.transform.translation.x
        target_point.y = transform.transform.translation.y
        target_point.z = transform.transform.translation.z
        
        # กำหนดพื้นที่เป้าหมายเป็นทรงกลมขนาดเล็ก (Tolerance Sphere)
        bv = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01] # รัศมี 1cm
        bv.primitives.append(primitive)
        bv.primitive_poses.append(PoseStamped(pose=PoseStamped().pose).pose) # Pose อ้างอิงจากจุดศูนย์กลาง Constraint Region
        
        pc.constraint_region = bv
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        # กำหนดจุดศูนย์กลางของ Constraint Region ให้เป็นจุดเป้าหมาย
        bv.primitive_poses[0].position = target_point
        
        c.position_constraints.append(pc)

        # 2. Orientation Constraint (ข้อจำกัดด้านการหมุน)
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.end_effector_link
        oc.orientation = transform.transform.rotation # ให้หมุนตาม Tag
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info("Sending goal to MoveIt...")
        self.action_client.wait_for_server()
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Callback เมื่อ Server ตอบรับ Goal
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Callback เมื่อทำงานเสร็จ
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    if len(sys.argv) < 2:
        print("Usage: test_gripper_to_tag.py <tag_name>")
        return

    tag_name = sys.argv[1]  # รับชื่อ Tag
    
    node = GripperToTag()
    node.send_goal(tag_name)  # ส่ง Goal
    rclpy.spin(node)  # หมุน loop

if __name__ == '__main__':
    main()
