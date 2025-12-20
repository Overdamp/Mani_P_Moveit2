#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped  # นำเข้า message types
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume  # นำเข้า message types สำหรับข้อจำกัด
from moveit_msgs.action import MoveGroup, ExecuteTrajectory  # นำเข้า action definition
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition
from shape_msgs.msg import SolidPrimitive  # นำเข้า message types สำหรับรูปทรง
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys
import math  # นำเข้า math
import time  # นำเข้า time

class Coarse2FineTask(Node):

    def __init__(self):
        super().__init__('coarse2fine_task')  # สร้าง Node ชื่อ 'coarse2fine_task'
        
        # --- CONFIG ---
        self.arm_group_name = "arm"       # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"         # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"     # ชื่อ frame อ้างอิง
        
        # การตั้งค่า Coarse (หยาบ)
        self.coarse_offset_z = 0.25  # ระยะห่าง 25cm สำหรับการเข้าหาครั้งแรก
        
        # การตั้งค่า Fine (ละเอียด)
        self.fine_tolerance = 0.002 # ความคลาดเคลื่อนที่ยอมรับได้ 2mm
        self.fine_max_step = 0.02   # ระยะขยับสูงสุดต่อครั้ง 2cm
        self.fine_max_retries = 10  # จำนวนครั้งสูงสุดที่พยายามปรับละเอียด
        # --------------

        # MoveIt Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')  # Client สำหรับสั่ง MoveGroup
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')  # Client สำหรับสั่ง Execute Trajectory
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')  # Client สำหรับคำนวณ Cartesian Path

        # TF Buffer และ Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Coarse-to-Fine Task Ready.')  # แสดงข้อความพร้อมทำงาน

    # =========================================================================
    # STEP 1: COARSE APPROACH (การเข้าหาแบบหยาบด้วย MoveIt Constraints)
    # =========================================================================
    def step_1_coarse_approach(self, tag_frame_id):
        self.get_logger().info(f"🚀 [STEP 1] Coarse Approach to {tag_frame_id} (Offset {self.coarse_offset_z}m)...")

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_group_name  # ระบุกลุ่มแขนกล
        goal_msg.request.num_planning_attempts = 10  # จำนวนครั้งที่พยายามวางแผน
        goal_msg.request.allowed_planning_time = 5.0  # เวลาที่อนุญาตให้วางแผน
        goal_msg.request.max_velocity_scaling_factor = 0.1  # ความเร็ว 10%
        goal_msg.request.max_acceleration_scaling_factor = 0.1  # ความเร่ง 10%
        
        # กำหนด Workspace
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        constraints.name = f"Approach_{tag_frame_id}"

        # Position Constraint (ทรงกลมรอบเป้าหมาย)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = tag_frame_id  # อ้างอิง Tag
        pos_con.link_name = self.ee_link
        pos_con.weight = 1.0
        
        region = BoundingVolume()
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005]  # รัศมี 5mm
        region.primitives.append(s)
        
        target_pose = Pose()
        target_pose.position.z = self.coarse_offset_z # ระยะ Offset จาก Tag
        target_pose.orientation.w = 1.0
        
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region
        
        # Orientation Constraint (จัดทิศทางให้ตรงกับ Tag)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = tag_frame_id
        ori_con.link_name = self.ee_link
        
        # เราต้องการให้มือจับ "หันหน้า" เข้าหา Tag
        # อ้างอิง logic จาก approach_tag_smart.py:
        # q_x = 0.0, q_y = 1.0, q_z = 0.0, q_w = 0.0 (สำหรับ Roll 0 องศา)
        # ค่านี้จะหมุน base orientation (ที่หันหน้าเข้าหา Tag) ให้ถูกต้อง
        
        ori_con.orientation.x = 0.0
        ori_con.orientation.y = 1.0
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        
        ori_con.absolute_x_axis_tolerance = 0.5 # ยอมรับความคลาดเคลื่อนได้มากหน่อย
        ori_con.absolute_y_axis_tolerance = 0.5 # ยอมรับความคลาดเคลื่อนได้มากหน่อย
        ori_con.absolute_z_axis_tolerance = 0.1 # Roll ต้องแม่นยำ
        ori_con.weight = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        # ส่ง Goal ไปยัง MoveGroup Action Server
        self._move_group_client.wait_for_server()
        future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Coarse Goal Rejected!')  # ถูกปฏิเสธ
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('✅ Coarse Approach Complete.')  # สำเร็จ
            return True
        else:
            self.get_logger().error(f'❌ Coarse Approach Failed: {result.error_code.val}')  # ล้มเหลว
            return False

    # =========================================================================
    # STEP 2: FINE ALIGNMENT (การปรับละเอียดด้วย Visual Servoing)
    # =========================================================================
    def step_2_fine_alignment(self, tag_frame_id):
        self.get_logger().info(f"🎯 [STEP 2] Fine Alignment to {tag_frame_id}...")
        
        for attempt in range(self.fine_max_retries):
            # 1. หาค่า Error (ระยะห่างจาก Tag)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.ee_link, tag_frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception:
                self.get_logger().warn(f"⚠️ Cannot see {tag_frame_id}")  # มองไม่เห็น Tag
                return False

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist_error = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().info(f"   Iter {attempt+1}: Error X={dx:.4f}, Y={dy:.4f} (Dist={dist_error:.4f})")

            if dist_error < self.fine_tolerance:
                self.get_logger().info("✅ Aligned!")  # ตรงแล้ว
                return True

            # 2. คำนวณระยะที่จะขยับ (Step)
            step_x, step_y = dx, dy
            if dist_error > self.fine_max_step:
                # ถ้า error มากกว่า step สูงสุด ให้ scale ลงมา
                scale = self.fine_max_step / dist_error
                step_x *= scale
                step_y *= scale

            # 3. สั่งเคลื่อนที่ (Relative Move)
            if not self.move_relative(step_x, step_y, 0.0):
                return False
            
            time.sleep(0.5) # รอให้นิ่ง

        self.get_logger().warn("❌ Max retries reached for fine alignment.")  # ครบจำนวนครั้งแล้วยังไม่ตรง
        return False

    def move_relative(self, x, y, z):
        # ดึงค่า Pose ปัจจุบัน
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            return False

        # หมุน Vector การเคลื่อนที่ให้ตรงกับ Base Frame
        qx, qy, qz, qw = t_base.transform.rotation.x, t_base.transform.rotation.y, t_base.transform.rotation.z, t_base.transform.rotation.w
        vx, vy, vz = x, y, z
        # สูตร Rotate Vector ด้วย Quaternion
        rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qz*qw)*vy + (2*qx*qz + 2*qy*qw)*vz
        ry = (2*qx*qy + 2*qz*qw)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qx*qw)*vz
        rz = (2*qx*qz - 2*qy*qw)*vx + (2*qy*qz + 2*qx*qw)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz

        # สร้าง Pose เป้าหมาย
        target_pose = Pose()
        target_pose.position.x = t_base.transform.translation.x + rx
        target_pose.position.y = t_base.transform.translation.y + ry
        target_pose.position.z = t_base.transform.translation.z + rz
        target_pose.orientation = t_base.transform.rotation

        # วางแผนและสั่งเคลื่อนที่ (Cartesian Path)
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = [target_pose]
        req.max_step = 0.01
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        self._execute_client.wait_for_server()
        send_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted: return False
        
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return True

    # =========================================================================
    # STEP 3: CONFIRMATION (ยืนยันความพร้อม)
    # =========================================================================
    def step_3_confirm(self):
        self.get_logger().info("⏳ Waiting for 5 seconds...")
        time.sleep(5.0)
        self.get_logger().info("\n" + "="*40)
        self.get_logger().info("   🤖 READY TO PICK! 📦   ")
        self.get_logger().info("="*40 + "\n")

    def run(self, tag_id):
        # ลำดับการทำงานหลัก
        if self.step_1_coarse_approach(tag_id):  # 1. เข้าหาแบบหยาบ
            time.sleep(1.0)
            if self.step_2_fine_alignment(tag_id):  # 2. ปรับละเอียด
                self.step_3_confirm()  # 3. ยืนยัน

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run mani_p_moveit_config3 coarse2fine_task.py <TAG_ID>")
        return

    tag_id = sys.argv[1]  # รับ Tag ID
    node = Coarse2FineTask()
    node.run(tag_id)  # เริ่มทำงาน
    node.destroy_node()
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
