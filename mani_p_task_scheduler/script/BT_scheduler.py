#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion  # นำเข้า message types
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume  # นำเข้า message types สำหรับข้อจำกัด
from moveit_msgs.action import MoveGroup, ExecuteTrajectory  # นำเข้า action definition
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition
from shape_msgs.msg import SolidPrimitive  # นำเข้า message types สำหรับรูปทรง
from control_msgs.action import GripperCommand  # นำเข้า action สำหรับ Gripper
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys
import math  # นำเข้า math
import time  # นำเข้า time
from enum import Enum, auto  # นำเข้า Enum

# ==========================================
# 🌳 LIGHTWEIGHT BEHAVIOR TREE FRAMEWORK 🌳 (โครงสร้าง Behavior Tree แบบเบา)
# ==========================================

class NodeStatus(Enum):
    SUCCESS = auto()  # สถานะสำเร็จ
    FAILURE = auto()  # สถานะล้มเหลว
    RUNNING = auto()  # สถานะกำลังทำงาน

class TreeNode:
    def __init__(self, name):
        self.name = name  # ชื่อ Node
        self.status = NodeStatus.FAILURE  # สถานะเริ่มต้น

    def tick(self):
        raise NotImplementedError("Tick not implemented")  # ต้อง implement ในคลาสลูก

class Sequence(TreeNode):
    """ Runs children sequentially. Fails if any child fails. """
    # รันลูกตามลำดับ ถ้ามีตัวไหนล้มเหลว ก็จะล้มเหลวทันที
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children  # รายการ Node ลูก

    def tick(self):
        for child in self.children:
            result = child.tick()  # สั่งให้ลูกทำงาน
            if result != NodeStatus.SUCCESS:
                self.status = result
                return result  # ถ้าไม่สำเร็จ ส่งคืนสถานะทันที
        self.status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS  # ถ้าสำเร็จทุกตัว ส่งคืน Success

class Selector(TreeNode):
    """ Runs children sequentially. Succeeds if any child succeeds. """
    # รันลูกตามลำดับ ถ้ามีตัวไหนสำเร็จ ก็จะสำเร็จทันที
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeStatus.SUCCESS:
                self.status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS  # ถ้าสำเร็จ ส่งคืน Success
            if result == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING  # ถ้ากำลังทำงาน ส่งคืน Running
        self.status = NodeStatus.FAILURE
        return NodeStatus.FAILURE  # ถ้าล้มเหลวทุกตัว ส่งคืน Failure

class Action(TreeNode):
    """ Leaf node that performs a task. """
    # Node ใบไม้ที่ทำงานจริง
    def __init__(self, name, action_func):
        super().__init__(name)
        self.action_func = action_func  # ฟังก์ชันที่จะให้ทำงาน

    def tick(self):
        # print(f"   [Action] {self.name}...")
        if self.action_func():  # เรียกใช้ฟังก์ชัน
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
        else:
            self.status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

# ==========================================
# 🤖 ROBOT BEHAVIOR NODE 🤖 (Node ควบคุมพฤติกรรมหุ่นยนต์)
# ==========================================

class PickAndPlaceBT(Node):

    def __init__(self):
        super().__init__('bt_scheduler')  # สร้าง Node ชื่อ 'bt_scheduler'
        
        # --- CONFIG ---
        self.arm_group_name = "arm"      # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"        # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"    # ชื่อ frame อ้างอิง
        self.target_tag = "tag2"         # Tag เป้าหมายเริ่มต้น
        
        self.approach_dist = 0.25        # ระยะเข้าหา
        self.grasp_dist = 0.15           # ระยะจับ
        self.align_tol = 0.002           # ความคลาดเคลื่อนที่ยอมรับได้
        # --------------

        # Clients (สร้าง Client สำหรับเรียกใช้ Service/Action ต่างๆ)
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # TF (ตัวจัดการ Transform)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('🌳 BT Scheduler Ready.')

    # --- ACTION FUNCTIONS (Return True/False) --- (ฟังก์ชันการทำงาน ส่งคืน True/False)

    def check_system(self):
        # ตรวจสอบความพร้อมของระบบ
        if not self._move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MoveGroup...")
            return False
        return True

    def find_tag(self):
        # ค้นหา Tag
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.target_tag, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            self.get_logger().info(f"✅ Found {self.target_tag}")
            return True
        except:
            self.get_logger().warn(f"Searching for {self.target_tag}...")
            return False

    def approach_tag(self):
        # เข้าหา Tag
        self.get_logger().info(f"🚀 Approaching {self.target_tag}...")
        
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
        constraints.name = f"Approach_{self.target_tag}"

        # Position Constraint (ข้อจำกัดตำแหน่ง)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = self.target_tag
        pos_con.link_name = self.ee_link
        pos_con.weight = 1.0
        region = BoundingVolume()
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005]
        region.primitives.append(s)
        target_pose = Pose()
        target_pose.position.z = float(self.approach_dist)
        target_pose.orientation.w = 1.0
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region

        # Orientation Constraint (ข้อจำกัดการหมุน)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = self.target_tag
        ori_con.link_name = self.ee_link
        ori_con.absolute_x_axis_tolerance = 0.2
        ori_con.absolute_y_axis_tolerance = 0.2
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        ori_con.orientation.w = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted: return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val == 1

    def visual_servo(self):
        # ปรับตำแหน่งละเอียดด้วย Visual Servoing
        self.get_logger().info("🎯 Visual Servoing...")
        for _ in range(5):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.ee_link, self.target_tag, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            except: return False

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist = math.sqrt(dx*dx + dy*dy)

            if dist < self.align_tol:
                self.get_logger().info("✅ Aligned!")
                return True

            max_step = 0.02
            if dist > max_step:
                scale = max_step / dist
                dx *= scale
                dy *= scale
            
            self.move_relative(dx, dy, 0.0)
            time.sleep(0.5)
        
        return False

    def open_gripper(self):
        # สั่งเปิดมือจับ
        self.get_logger().info("👐 Opening Gripper")
        return self.control_gripper(open=True)

    def close_gripper(self):
        # สั่งปิดมือจับ
        self.get_logger().info("✊ Closing Gripper")
        return self.control_gripper(open=False)

    def push_forward(self):
        # ดันไปข้างหน้า
        self.get_logger().info("⬇️ Pushing Forward")
        return self.move_linear(self.grasp_dist)

    def pull_back(self):
        # ดึงกลับ
        self.get_logger().info("⬆️ Pulling Back")
        return self.move_linear(-self.grasp_dist)

    # --- HELPERS --- (ฟังก์ชันช่วย)

    def move_relative(self, x, y, z):
        # เคลื่อนที่สัมพัทธ์
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except: return False

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
        # เคลื่อนที่เชิงเส้น
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except: return False

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

        if response.error_code.val != 1: return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_goal_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted: return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.error_code.val == 1

    def control_gripper(self, open=True):
        # ควบคุม Gripper
        goal = GripperCommand.Goal()
        goal.command.position = 0.03 if open else -0.01
        goal.command.max_effort = 100.0
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return True

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    # CLI Argument (รับค่าจาก Command Line)
    target_tag = "tag2"
    if len(sys.argv) > 1:
        target_tag = sys.argv[1]

    # Create Node (สร้าง Node)
    robot = PickAndPlaceBT()
    robot.target_tag = target_tag

    # ==========================
    # 🌳 BUILD THE TREE 🌳 (สร้าง Behavior Tree)
    # ==========================
    
    # 1. Grasp Sequence (Push -> Close -> Pull) (ลำดับการจับ: ดัน -> ปิด -> ดึง)
    grasp_seq = Sequence("GraspSequence", [
        Action("OpenGripper", robot.open_gripper),
        Action("PushForward", robot.push_forward),
        Action("CloseGripper", robot.close_gripper),
        Action("PullBack", robot.pull_back)
    ])

    # 2. Main Sequence (ลำดับหลัก)
    root = Sequence("MainTask", [
        Action("CheckSystem", robot.check_system),
        Action("FindTag", robot.find_tag),
        Action("ApproachTag", robot.approach_tag),
        Action("VisualServo", robot.visual_servo),
        grasp_seq
    ])

    # ==========================
    # 🏃 RUN THE TREE 🏃 (รัน Tree)
    # ==========================
    
    robot.get_logger().info("--- STARTING BEHAVIOR TREE ---")
    
    # Simple Loop (Tick until Success or Failure) (วนลูปจนกว่าจะสำเร็จหรือล้มเหลว)
    while rclpy.ok():
        status = root.tick()
        
        if status == NodeStatus.SUCCESS:
            robot.get_logger().info("✅ TREE FINISHED: SUCCESS")
            break
        elif status == NodeStatus.FAILURE:
            robot.get_logger().error("❌ TREE FINISHED: FAILURE")
            break
        
        # If RUNNING (not implemented in this simple version, but good practice)
        time.sleep(0.1)

    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
