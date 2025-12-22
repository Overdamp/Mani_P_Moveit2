#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.action import ActionServer  # นำเข้า ActionServer
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.callback_groups import ReentrantCallbackGroup  # นำเข้า Callback Group แบบ Reentrant
from rclpy.executors import MultiThreadedExecutor  # นำเข้า MultiThreadedExecutor

from mani_p_actions.action import MoveToShelf  # นำเข้า Action Interface ที่สร้างเอง
from geometry_msgs.msg import PoseStamped, Pose  # นำเข้า message types
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import tf2_geometry_msgs  # นำเข้าไลบรารีแปลง TF สำหรับ Geometry Messages

# MoveIt
from moveit_msgs.action import MoveGroup  # นำเข้า action definition สำหรับ MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint  # นำเข้า message types สำหรับข้อจำกัด
from rclpy.action import ActionClient  # นำเข้า ActionClient

# --- Internal Helper Class (คลาสช่วยคำนวณภายใน) ---
class ShelfCalculator:
    def __init__(self, tf_buffer, logger):
        self.tf_buffer = tf_buffer
        self.logger = logger
        
        # Configuration (การตั้งค่า)
        self.ref_tag = "tag11" # Top Center Tag (Tag อ้างอิงตรงกลางบน)
        self.row_spacing = 0.23  # 23cm spacing (ระยะห่างระหว่างแถว)
        self.col_spacing = 0.23  # 23cm spacing (ระยะห่างระหว่างคอลัมน์)
        self.standoff_dist = 0.314 # Measured Z (ระยะห่างแนวแกน Z ที่วัดได้)
        
        # Calibrated Base Offsets for Row 2, Col 2 (Middle Center) (ค่า Offset ฐานที่ Calibrate แล้วสำหรับแถว 2 คอลัมน์ 2)
        # Updated from User Calibration (2025-12-13 - Evening)
        self.base_x_offset = 0.0
        self.base_y_offset = -0.445
        self.standoff_dist = 0.10 # Z-offset (In/Out) - User updated to 0.15 (ระยะห่างแนวแกน Z ที่ผู้ใช้อัปเดต)

    def get_target_pose(self, row, col, timeout_sec=5.0):
        """
        Calculate target pose in Base_link frame. (คำนวณตำแหน่งเป้าหมายใน Base_link frame)
        Row: 1 (Top), 2 (Middle), 3 (Bottom)
        Col: 1 (Left), 2 (Center), 3 (Right)
        """
        # 1. Calculate offsets in Tag Frame (คำนวณ Offset ใน Tag Frame)
        # Base is Row 2, Col 2 (Middle Center)
        
        # Column Logic:
        # Col 1 (Left) -> base_x - spacing
        # Col 2 (Center) -> base_x
        # Col 3 (Right) -> base_x + spacing
        if col == 1:
            x_offset = self.base_x_offset - self.col_spacing
        elif col == 2:
            x_offset = self.base_x_offset
        elif col == 3:
            x_offset = self.base_x_offset + self.col_spacing
        else:
            self.logger.error("Invalid Column! Use 1, 2, or 3.")
            return None

        # Row Logic:
        # Calibration Y = -0.428 (Negative)
        # Tag Y points UP.
        # Row 1 (Top) -> base_y + spacing (Higher Y)
        # Row 2 (Middle) -> base_y
        # Row 3 (Bottom) -> base_y - spacing (Lower Y)
        
        y_offset = self.base_y_offset + (2 - row) * self.row_spacing
        
        z_offset = self.standoff_dist
        
        self.logger.info(f"🎯 Target relative to Tag: X={x_offset}, Y={y_offset}, Z={z_offset}")
        
        # 2. Create Pose in Tag Frame (สร้าง Pose ใน Tag Frame)
        target_pose_tag = PoseStamped()
        target_pose_tag.header.frame_id = self.ref_tag
        target_pose_tag.header.stamp = rclpy.time.Time().to_msg() # Will be updated by transform
        target_pose_tag.pose.position.x = x_offset
        target_pose_tag.pose.position.y = y_offset
        target_pose_tag.pose.position.z = z_offset
        
        # Orientation: Perfect Alignment (X Up, Z In) (การหมุน: จัดแนวให้สมบูรณ์)
        # Quaternion for 180 deg rotation around (1,1,0) axis
        target_pose_tag.pose.orientation.x = 0.7071
        target_pose_tag.pose.orientation.y = 0.7071
        target_pose_tag.pose.orientation.z = 0.0
        target_pose_tag.pose.orientation.w = 0.0
        
        # 3. Transform to Base_link (แปลงเป็น Base_link)
        try:
            if not self.tf_buffer.can_transform('Base_link', self.ref_tag, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=timeout_sec)):
                self.logger.warn(f"⚠️ Cannot find transform Base_link -> {self.ref_tag} after {timeout_sec}s")
                return None
                
            transform = self.tf_buffer.lookup_transform('Base_link', self.ref_tag, rclpy.time.Time())
            
            # Update stamp to match transform time to avoid extrapolation errors
            target_pose_tag.header.stamp = transform.header.stamp
            
            # Fix: do_transform_pose expects a Pose object, not PoseStamped
            target_pose_base_pose = tf2_geometry_msgs.do_transform_pose(target_pose_tag.pose, transform)
            
            # Create stamped pose for return/publishing
            target_pose_base_stamped = PoseStamped()
            target_pose_base_stamped.header.frame_id = 'Base_link'
            target_pose_base_stamped.header.stamp = transform.header.stamp
            target_pose_base_stamped.pose = target_pose_base_pose
            
            # Log Tag Pose (for debugging)
            tag_pose_base = tf2_geometry_msgs.do_transform_pose(PoseStamped(pose=Pose(), header=target_pose_tag.header).pose, transform)
            self.logger.info(f"🏷️ Tag Pose (Base_link): X={tag_pose_base.position.x:.3f}, Y={tag_pose_base.position.y:.3f}, Z={tag_pose_base.position.z:.3f}")
            
            # Log Target Pose
            self.logger.info(f"📍 Target Pose (Base_link): X={target_pose_base_stamped.pose.position.x:.3f}, Y={target_pose_base_stamped.pose.position.y:.3f}, Z={target_pose_base_stamped.pose.position.z:.3f}")
            
            # Calculate Distance from Base (0,0,0) to Target
            import math
            dist = math.sqrt(target_pose_base_stamped.pose.position.x**2 + target_pose_base_stamped.pose.position.y**2 + target_pose_base_stamped.pose.position.z**2)
            self.logger.info(f"📏 Distance from Base to Target: {dist:.3f} meters")
            
            return target_pose_base_stamped
            
        except Exception as e:
            self.logger.error(f"Transform Error: {e}")
            return None

# --- Main Action Server (Action Server หลัก) ---
class ShelfActionServer(Node):
    def __init__(self):
        super().__init__('shelf_action_server')  # สร้าง Node ชื่อ 'shelf_action_server'
        
        self.callback_group = ReentrantCallbackGroup()  # สร้าง Callback Group เพื่อให้ทำงานขนานกันได้
        
        # สร้าง Action Server สำหรับ MoveToShelf
        self._action_server = ActionServer(
            self,
            MoveToShelf,
            'move_to_shelf',
            self.execute_callback,
            callback_group=self.callback_group)
            
        # TF Setup (ตั้งค่า TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # MoveIt Action Client (สร้าง Client สำหรับ MoveGroup)
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.callback_group)
        
        # Internal Calculator (ตัวคำนวณภายใน)
        self.calculator = ShelfCalculator(self.tf_buffer, self.get_logger())
        
        # Publisher for visualization (Added feature from navigator) (Publisher สำหรับแสดงผลใน RViz)
        self.goal_pub = self.create_publisher(PoseStamped, 'shelf_goal_pose', 10)
        
        # Joint State Subscription (รับค่า Joint State)
        from sensor_msgs.msg import JointState
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        self.get_logger().info("📦 Shelf Action Server Ready (Consolidated Version)")

    def joint_callback(self, msg):
        # Callback สำหรับเก็บค่า Joint ปัจจุบัน
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def log_status(self):
        # 1. Get TCP Pose (ดึงตำแหน่ง TCP)
        try:
            if self.tf_buffer.can_transform('Base_link', 'tcp_link', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('Base_link', 'tcp_link', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                
                # 2. Get Joints (ดึงค่า Joint)
                joints_str = ", ".join([f"{k}: {v:.3f}" for k, v in self.current_joints.items() if 'J' in k or 'palm' in k])
                
                print(f"📍 TCP: [{x:.3f}, {y:.3f}, {z:.3f}] | 🦾 Joints: {{{joints_str}}}")
        except Exception as e:
            pass

    async def execute_callback(self, goal_handle):
        # Callback หลักเมื่อได้รับ Goal
        self.get_logger().info('Executing goal...')
        print("DEBUG: execute_callback started") # Force print
        
        feedback_msg = MoveToShelf.Feedback()
        feedback_msg.status = "Planning"
        goal_handle.publish_feedback(feedback_msg)
        result = MoveToShelf.Result()
        
        row = goal_handle.request.row
        col = goal_handle.request.col
        
        print(f"DEBUG: Request Row={row}, Col={col}")
        
        # 1. Calculate Target Pose (คำนวณตำแหน่งเป้าหมาย)
        target_pose_base = self.calculator.get_target_pose(row, col)
        
        if target_pose_base is None:
            self.get_logger().error("Could not calculate target pose")
            print("DEBUG: target_pose_base is None!")
            goal_handle.abort()
            return MoveToShelf.Result(success=False, message="Calculation Failed")
            
        print(f"DEBUG: Target Pose Calculated: {target_pose_base.pose.position.x}, {target_pose_base.pose.position.y}, {target_pose_base.pose.position.z}")
            
        # Visualize Target (แสดงผลเป้าหมาย)
        self.goal_pub.publish(target_pose_base)
        self.get_logger().info("📡 Published target to /shelf_goal_pose for visualization")
            
        feedback_msg.status = "Planning path..."
        goal_handle.publish_feedback(feedback_msg)
        
        # 2. Execute Move (สั่งเคลื่อนที่)
        success = await self.move_to_pose(target_pose_base, goal_handle)
        
        if success:
            result.success = True
            result.message = "Arrived at shelf slot."
            goal_handle.succeed()
        else:
            result.success = False
            result.message = "Motion planning failed."
            goal_handle.abort()
            
        return result

    async def move_to_pose(self, pose_stamped, goal_handle_server):
        # ฟังก์ชันสั่งเคลื่อนที่ไปยัง Pose
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 20 # Increased attempts (เพิ่มจำนวนครั้งที่พยายาม)
        goal_msg.request.allowed_planning_time = 10.0 # Increased time (เพิ่มเวลาที่ให้)
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.planner_id = "RRTConnectkConfigDefault" # Force a robust planner (บังคับใช้ Planner ที่ทนทาน)
        
        # Workspace Parameters (พารามิเตอร์พื้นที่ทำงาน)
        goal_msg.request.workspace_parameters.header.frame_id = 'Base_link'
        goal_msg.request.workspace_parameters.min_corner.x = -3.0
        goal_msg.request.workspace_parameters.min_corner.y = -3.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 3.0
        goal_msg.request.workspace_parameters.max_corner.y = 3.0
        goal_msg.request.workspace_parameters.max_corner.z = 3.0
        
        # Position Constraint (ข้อจำกัดตำแหน่ง)
        pcm = PositionConstraint()
        pcm.header.frame_id = 'Base_link'
        pcm.link_name = 'tcp_link'
        pcm.target_point_offset.x = 0.0
        pcm.target_point_offset.y = 0.0
        pcm.target_point_offset.z = 0.0
        
        # Create a small box for the target position (Relaxed to 5cm) (สร้างกล่องเล็กๆ สำหรับตำแหน่งเป้าหมาย)
        from shape_msgs.msg import SolidPrimitive
        pcm.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05]))
        pcm.constraint_region.primitive_poses.append(pose_stamped.pose)
        pcm.weight = 1.0
        
        # Orientation Constraint (Strictly Enforced) (ข้อจำกัดการหมุน - บังคับใช้อย่างเคร่งครัด)
        ocm = OrientationConstraint()
        ocm.header.frame_id = 'Base_link'
        ocm.link_name = 'tcp_link'
        ocm.orientation = pose_stamped.pose.orientation
        ocm.absolute_x_axis_tolerance = 0.1 # Strict
        ocm.absolute_y_axis_tolerance = 0.1 # Strict
        ocm.absolute_z_axis_tolerance = 0.1 # Strict
        ocm.weight = 1.0
        
        # Add BOTH Position and Orientation Constraints (เพิ่มทั้งข้อจำกัดตำแหน่งและการหมุน)
        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
        
        self.get_logger().info(f"🚀 Sending Goal to MoveIt:")
        self.get_logger().info(f"   Frame: {pose_stamped.header.frame_id}")
        self.get_logger().info(f"   Pos: {pose_stamped.pose.position.x:.3f}, {pose_stamped.pose.position.y:.3f}, {pose_stamped.pose.position.z:.3f}")
        self.get_logger().info(f"   Ori: {pose_stamped.pose.orientation.x:.3f}, {pose_stamped.pose.orientation.y:.3f}, {pose_stamped.pose.orientation.z:.3f}, {pose_stamped.pose.orientation.w:.3f}")
        
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance (รอการตอบรับ Goal)
        while not send_goal_future.done():
            if goal_handle_server.is_cancel_requested:
                self.get_logger().info('Goal canceled before acceptance')
                goal_handle_server.canceled()
                return False
            import time
            time.sleep(0.1)

        goal_handle_moveit = send_goal_future.result()
        
        if not goal_handle_moveit.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        result_future = goal_handle_moveit.get_result_async()
        
        # Wait for result with cancellation check (รอผลลัพธ์และตรวจสอบการยกเลิก)
        while not result_future.done():
            if goal_handle_server.is_cancel_requested:
                self.get_logger().info('Cancellation requested. Cancelling MoveIt goal...')
                cancel_future = goal_handle_moveit.cancel_goal_async()
                while not cancel_future.done():
                    import time
                    time.sleep(0.01)
                goal_handle_server.canceled()
                return False
            
            # Log Status while moving (Log สถานะขณะเคลื่อนที่)
            self.log_status()
            
            import time
            time.sleep(0.5) # Print every 0.5s

        result = result_future.result()
        
        if result.result.error_code.val == 1: # SUCCESS
            return True
        else:
            self.get_logger().error(f"MoveIt Error Code: {result.result.error_code.val}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ShelfActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
