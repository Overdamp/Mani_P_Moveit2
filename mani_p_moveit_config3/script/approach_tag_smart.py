#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
import sys # <--- พระเอกของเรา (ใช้รับค่าจาก command line)
import math
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from geometry_msgs.msg import PoseStamped

class SmartApproacher(Node):

    def __init__(self):
        super().__init__('smart_approacher')
        
        # --- ⚙️ CONFIG ⚙️ ---
        self.arm_group_name = "arm"      
        self.ee_link = "tcp_link"        # แนะนำให้ใช้ tcp_link (ปลายมือจริง)
        self.base_frame = "Base_link"    
        # --------------------

        # Action Client for PTP (Joint/Pose Goal)
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Action Client for Trajectory Execution
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        # Service Client for Cartesian Path
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('CLI Approacher Ready (Smart & Cartesian).')



    def go_to_tag_offset(self, tag_frame_id, offset_z=0.15, roll_deg=0.0):
        """ สั่งแขนไปที่ Offset ของ Tag พร้อมหมุน Roll (degrees) """
        
        # 1. สร้าง Goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1 
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # 2. Workspace
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        # 3. Constraints
        constraints = Constraints()
        constraints.name = f"Approach_{tag_frame_id}"

        # --- Position (อ้างอิง Tag) ---
        pos_con = PositionConstraint()
        pos_con.header.frame_id = tag_frame_id # 🎯 Frame เป้าหมายคือ Tag ที่รับเข้ามา
        pos_con.link_name = self.ee_link
        pos_con.weight = 1.0

        region = BoundingVolume()
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005] 
        region.primitives.append(s)

        target_pose = Pose()
        target_pose.position.x = 0.0
        target_pose.position.y = 0.0
        target_pose.position.z = float(offset_z) # 🎯 ระยะห่างที่รับเข้ามา
        target_pose.orientation.w = 1.0 
        
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region
        
        # --- Orientation (อ้างอิง Tag) ---
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = tag_frame_id
        ori_con.link_name = self.ee_link
        
        # คำนวณ Quaternion สำหรับ Roll Offset
        # Base Orientation (หันหน้าเข้าหา Tag): (x=0, y=1, z=0, w=0)
        # เราต้องการหมุนรอบแกน Z ของ Gripper (ซึ่งคือแกน Y ของ Tag ใน Base Orient นี้)
        # Math: q_new = q_base * q_roll_z
        # q_base = (0, 1, 0, 0)
        # q_roll_z = (0, 0, sin(a/2), cos(a/2))
        # Result q_new = (sin(a/2), cos(a/2), 0, 0)
        
        half_angle = math.radians(roll_deg) / 2.0
        q_x = math.sin(half_angle)
        q_y = math.cos(half_angle)
        
        ori_con.orientation.x = q_x
        ori_con.orientation.y = q_y
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        
        ori_con.absolute_x_axis_tolerance = 0.2 # Somewhat relaxed
        ori_con.absolute_y_axis_tolerance = 0.2 # Somewhat relaxed
        ori_con.absolute_z_axis_tolerance = 0.1 # Strict Roll (Must align)
        ori_con.weight = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        # 4. Send Action
        self.get_logger().info(f"🚀 Sending Goal: Go to {tag_frame_id} (Offset {offset_z}m, Roll {roll_deg}°)...")
        
        self._action_client.wait_for_server()
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected!')
            return

        self.get_logger().info('⏳ Moving...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info(f'✅ SUCCESS: Reached {tag_frame_id}!')
        else:
            self.get_logger().error(f'❌ FAILED: Error Code {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    
    # --- 🎮 ส่วนรับค่าจาก Command Line (CLI) 🎮 ---
    
    # sys.argv[0] คือชื่อไฟล์ script
    # sys.argv[1] คือ argument ตัวแรก (ชื่อ Tag)
    # sys.argv[2] คือ argument ตัวที่สอง (ระยะห่าง - optional)

    if len(sys.argv) < 2:
        print("\n⚠️  Usage Error!")
        print("   Syntax: ros2 run <pkg> approach_tag_smart.py <TAG_NAME> [DISTANCE] [ROLL_DEG]")
        print("   Example: ros2 run ... approach_tag_smart.py tag2")
        print("   Example: ros2 run ... approach_tag_smart.py tag2 0.15 -90")
        print("   Example: ros2 run ... approach_tag_smart.py cartesian 0.15 (Move Forward)\n")
        return

    # เริ่มทำงาน
    approacher = SmartApproacher()
    approacher.go_to_tag_offset(target_tag, distance, roll_deg)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()