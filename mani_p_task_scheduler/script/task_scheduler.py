#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion  # นำเข้า message types
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume  # นำเข้า message types สำหรับข้อจำกัด
from moveit_msgs.action import MoveGroup, ExecuteTrajectory  # นำเข้า action definition
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition
from shape_msgs.msg import SolidPrimitive  # นำเข้า message types สำหรับรูปทรง
from control_msgs.action import GripperCommand  # นำเข้า action definition สำหรับ Gripper
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys
import math  # นำเข้า math
import time  # นำเข้า time
from enum import Enum, auto  # นำเข้า Enum

class State(Enum):
    INIT = auto()           # สถานะเริ่มต้น
    SCAN = auto()           # สถานะสแกนหา Tag
    APPROACH = auto()       # สถานะเข้าหา Tag
    ALIGN = auto()          # สถานะจัดแนว
    GRASP_APPROACH = auto() # สถานะเข้าจับ (ระยะใกล้)
    GRASP_ACTION = auto()   # สถานะจับ
    RETREAT = auto()        # สถานะถอยหลัง
    DONE = auto()           # สถานะเสร็จสิ้น
    FAIL = auto()           # สถานะล้มเหลว

class PickAndPlaceScheduler(Node):

    def __init__(self):
        super().__init__('task_scheduler')  # สร้าง Node ชื่อ 'task_scheduler'
        
        # --- ⚙️ CONFIG ⚙️ ---
        self.arm_group_name = "arm"      # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"        # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"    # ชื่อ frame อ้างอิง
        self.target_tag = "tag2"         # Default Target (Tag เป้าหมายเริ่มต้น)
        
        # Parameters (พารามิเตอร์)
        self.approach_distance = 0.25    # 25cm standoff (ระยะห่างสำหรับเข้าหา)
        self.grasp_distance = 0.15       # 15cm push (ระยะดันเข้าไปจับ)
        self.align_tolerance = 0.002     # 2mm alignment (ความคลาดเคลื่อนที่ยอมรับได้ในการจัดแนว)
        
        # --------------------

        # Clients (สร้าง Client สำหรับ Action และ Service ต่างๆ)
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # TF (ตั้งค่า TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('🤖 Pick & Place Scheduler Ready.')

    def run(self):
        """ Main FSM Loop (ลูปหลักของ Finite State Machine) """
        state = State.INIT
        
        while rclpy.ok():
            if state == State.INIT:
                self.get_logger().info("🔵 STATE: INIT")
                # Wait for clients? (รอให้ Action Server พร้อม)
                if not self._move_group_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("MoveGroup Action not available!")
                    state = State.FAIL
                    continue
                state = State.SCAN

            elif state == State.SCAN:
                self.get_logger().info(f"🔵 STATE: SCAN (Looking for {self.target_tag})")
                # Check if we can see the tag (ตรวจสอบว่ามองเห็น Tag หรือไม่)
                if self.check_tf(self.target_tag):
                    self.get_logger().info("   ✅ Tag Found!")
                    state = State.APPROACH
                else:
                    self.get_logger().warn("   ⚠️ Tag not visible. Retrying...")
                    time.sleep(1.0)

            elif state == State.APPROACH:
                self.get_logger().info("🔵 STATE: APPROACH (Coarse)")
                # เข้าหา Tag แบบหยาบ
                success = self.go_to_tag_smart(self.target_tag, self.approach_distance)
                if success:
                    state = State.ALIGN
                else:
                    self.get_logger().error("   ❌ Approach Failed.")
                    state = State.FAIL

            elif state == State.ALIGN:
                self.get_logger().info("🔵 STATE: ALIGN (Visual Servo)")
                # Use Eye-in-Hand to align (ใช้ Visual Servoing จัดแนว)
                success = self.visual_servo_align(self.target_tag)
                if success:
                    state = State.GRASP_APPROACH
                else:
                    self.get_logger().warn("   ⚠️ Alignment incomplete, but proceeding...")
                    state = State.GRASP_APPROACH # Proceed anyway? (ไปต่อแม้จัดแนวไม่สมบูรณ์?)

            elif state == State.GRASP_APPROACH:
                self.get_logger().info("🔵 STATE: GRASP APPROACH (Push)")
                self.control_gripper(open=True) # Open gripper first (กางมือก่อน)
                success = self.move_linear(self.grasp_distance) # ดันเข้าไป
                if success:
                    state = State.GRASP_ACTION
                else:
                    state = State.FAIL

            elif state == State.GRASP_ACTION:
                self.get_logger().info("🔵 STATE: GRASP ACTION")
                self.control_gripper(open=False) # Close gripper (หุบมือจับ)
                time.sleep(1.0) # Wait for grasp (รอให้จับแน่น)
                state = State.RETREAT

            elif state == State.RETREAT:
                self.get_logger().info("🔵 STATE: RETREAT")
                success = self.move_linear(-self.grasp_distance) # Pull back (ถอยหลังกลับ)
                if success:
                    state = State.DONE
                else:
                    state = State.FAIL

            elif state == State.DONE:
                self.get_logger().info("✅ MISSION COMPLETE!")
                break

            elif state == State.FAIL:
                self.get_logger().error("❌ MISSION FAILED!")
                break
            
            time.sleep(0.5)

    # --- ACTION IMPLEMENTATIONS (การทำงานของแต่ละ Action) ---

    def check_tf(self, target_frame):
        # ตรวจสอบว่ามี TF ของเป้าหมายหรือไม่
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return True
        except:
            return False

    def go_to_tag_smart(self, tag_frame, distance):
        """ Logic from approach_tag_smart.py (ตรรกะจาก approach_tag_smart.py) """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        constraints = Constraints()
        constraints.name = f"Approach_{tag_frame}"

        # Position (ข้อจำกัดตำแหน่ง)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = tag_frame
        pos_con.link_name = self.ee_link
        pos_con.weight = 1.0
        region = BoundingVolume()
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005]
        region.primitives.append(s)
        target_pose = Pose()
        target_pose.position.z = float(distance)
        target_pose.orientation.w = 1.0
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region

        # Orientation (ข้อจำกัดการหมุน)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = tag_frame
        ori_con.link_name = self.ee_link
        ori_con.absolute_x_axis_tolerance = 0.2
        ori_con.absolute_y_axis_tolerance = 0.2
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        ori_con.orientation.w = 1.0 # Identity (Face tag) (หันหน้าเข้าหา Tag)

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val == 1

    def visual_servo_align(self, tag_frame):
        """ Logic from visual_servo_align.py (ตรรกะจาก visual_servo_align.py) """
        for _ in range(5): # Max 5 attempts (พยายามสูงสุด 5 ครั้ง)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.ee_link, tag_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            except:
                return False

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.align_tolerance:
                return True

            # Limit step (จำกัดระยะการเคลื่อนที่ต่อครั้ง)
            max_step = 0.02
            if dist > max_step:
                scale = max_step / dist
                dx *= scale
                dy *= scale
            
            # Move relative (เคลื่อนที่สัมพัทธ์)
            self.move_relative(dx, dy, 0.0)
            time.sleep(0.5)
        
        return False # Timeout (หมดเวลา)

    def move_relative(self, x, y, z):
        """ Move TCP relative to itself (Cartesian) (เคลื่อนที่ TCP สัมพัทธ์กับตัวเอง) """
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except:
            return False

        # Rotate vector to base frame (หมุน Vector ไปยัง Base Frame)
        qx = t_base.transform.rotation.x
        qy = t_base.transform.rotation.y
        qz = t_base.transform.rotation.z
        qw = t_base.transform.rotation.w

        vx, vy, vz = x, y, z
        rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qz*qw)*vy + (2*qx*qz + 2*qy*qw)*vz
        ry = (2*qx*qy + 2*qz*qw)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qx*qw)*vz
        rz = (2*qx*qz - 2*qy*qw)*vx + (2*qy*qz + 2*qx*qw)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz

        target_pose = Pose()
        target_pose.position.x = t_base.transform.translation.x + rx
        target_pose.position.y = t_base.transform.translation.y + ry
        target_pose.position.z = t_base.transform.translation.z + rz
        target_pose.orientation = t_base.transform.rotation

        return self.execute_cartesian(target_pose)

    def move_linear(self, distance):
        """ Move forward/backward along Z axis (เคลื่อนที่หน้า/หลังตามแกน Z) """
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except:
            return False

        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        
        dx = 2 * (qx*qz + qy*qw) * distance
        dy = 2 * (qy*qz - qx*qw) * distance
        dz = (1 - 2 * (qx*qx + qy*qy)) * distance

        target_pose = Pose()
        target_pose.position.x = t.transform.translation.x + dx
        target_pose.position.y = t.transform.translation.y + dy
        target_pose.position.z = t.transform.translation.z + dz
        target_pose.orientation = t.transform.rotation

        return self.execute_cartesian(target_pose)

    def execute_cartesian(self, target_pose):
        # สั่งเคลื่อนที่แบบ Cartesian
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = [target_pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_goal_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val == 1

    def control_gripper(self, open=True):
        # ควบคุม Gripper
        goal = GripperCommand.Goal()
        # Adjust these values based on your gripper (0.01 = Open, -0.01 = Close ?)
        # Usually: 0.0 is closed, Max is open. Or vice versa.
        # Assuming: 0.03 = Open, 0.0 = Close
        goal.command.position = 0.03 if open else -0.01
        goal.command.max_effort = 100.0
        
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        # We don't strictly wait for result here to keep it simple, or we can.
        return True

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    # CLI Argument for Tag (รับ Argument Tag จาก Command Line)
    target_tag = "tag2"
    if len(sys.argv) > 1:
        target_tag = sys.argv[1]

    scheduler = PickAndPlaceScheduler()
    scheduler.target_tag = target_tag
    scheduler.run()
    
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
