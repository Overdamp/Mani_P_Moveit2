#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration

# TF2 & Geometry
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs # สำคัญ! ต้องมี module นี้เพื่อแปลง Pose
from geometry_msgs.msg import PoseStamped, Pose

# Interfaces
from mani_p_actions.action import VisualServo
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState

class VisualServoActionServer(Node):

    def __init__(self):
        super().__init__('visual_servo_action_server')
        
        # --- Configuration ---
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"       # ลิงค์ปลายมือ (End Effector)
        self.base_frame = "Base_link"   # ลิงค์ฐานหุ่นยนต์
        
        # ใช้ ReentrantCallbackGroup เพื่อให้ Action Client ทำงานซ้อนกับ Server ได้
        self.cb_group = ReentrantCallbackGroup()

        # 1. Action Server (รับคำสั่งจาก User/Behavior Tree)
        self._action_server = ActionServer(
            self,
            VisualServo,
            'visual_servo',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # 2. Clients เพื่อคุยกับ MoveIt
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        # 3. TF Listener Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4. Debugging Tools (Joint State)
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

        self.get_logger().info('✅ Visual Servo Action Server (Optimized) Ready.')

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def log_status(self):
        """ ปริ้นสถานะปัจจุบันเพื่อ Debug """
        try:
            if self.tf_buffer.can_transform(self.base_frame, self.ee_link, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
                x, y, z = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
                # เลือกโชว์เฉพาะ Joint ที่น่าสนใจ
                joints_str = ", ".join([f"{k}: {v:.3f}" for k, v in self.current_joints.items() if 'J' in k or 'palm' in k])
                print(f"📍 TCP: [{x:.3f}, {y:.3f}, {z:.3f}] | 🦾 Joints: {{{joints_str}}}")
        except Exception:
            pass

    async def execute_callback(self, goal_handle):
        tag_frame = goal_handle.request.tag_name
        tolerance = goal_handle.request.tolerance
        target_standoff = 0.25 # เมตร (ระยะห่างที่ต้องการ)
        
        self.get_logger().info(f"🎯 6DOF Aligning to '{tag_frame}' (Tol: {tolerance:.4f} m, Standoff: {target_standoff} m)...")
        
        max_retries = 10
        
        result = VisualServo.Result()
        dist_error = 0.0 # Initialize to avoid UnboundLocalError
        
        for attempt in range(max_retries):
            # 0. Check Cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("🛑 Goal Canceled")
                return VisualServo.Result(success=False, final_error=0.0)

            # 1. Get Tag Pose in Base Frame
            try:
                if not self.tf_buffer.can_transform(self.base_frame, tag_frame, rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                    self.get_logger().warn(f"⚠️ Cannot see {tag_frame}")
                    time.sleep(0.5)
                    continue
                    
                t_base_tag = self.tf_buffer.lookup_transform(self.base_frame, tag_frame, rclpy.time.Time())
                
                # 2. Calculate Target Pose (Tag + Standoff)
                # เราต้องการให้ TCP อยู่หน้า Tag ตามแกน Z ของ Tag
                # สร้าง Pose ของเป้าหมายใน Frame ของ Tag ก่อน
                target_pose_tag = PoseStamped()
                target_pose_tag.header.frame_id = tag_frame
                target_pose_tag.header.stamp = t_base_tag.header.stamp
                target_pose_tag.pose.position.z = target_standoff # ถอยออกมาตามแกน Z
                
                # Orientation:
                # เราต้องการให้ TCP หันหน้าเข้าหา Tag
                # ถ้า Tag Z ชี้ออกมา, และ TCP Z ชี้ออกจากมือ
                # เราต้องหมุน TCP ให้สวนทางกับ Tag (Rotate 180 รอบแกน X หรือ Y)
                # Quaternion for 180 deg rotation around X-axis: [1, 0, 0, 0] -> [0, 1, 0, 0] (x, y, z, w)
                # ลองใช้ Identity ก่อน (Orientation เดียวกับ Tag) แล้วดูว่ามือหันทางไหน
                # ปกติถ้า Tag แปะผนัง Z ชี้ออก, มือเรา Z ชี้ออก -> ต้องหมุน 180
                # Quaternion (x=1, y=0, z=0, w=0) คือหมุน 180 รอบแกน X
                target_pose_tag.pose.orientation.x = 1.0
                target_pose_tag.pose.orientation.y = 0.0
                target_pose_tag.pose.orientation.z = 0.0
                target_pose_tag.pose.orientation.w = 0.0
                
                # แปลง Target Pose กลับมาเป็น Base Frame
                target_pose_base = tf2_geometry_msgs.do_transform_pose(target_pose_tag.pose, t_base_tag)
                
                # สร้าง PoseStamped สำหรับ Base Frame
                target_pose_base_stamped = PoseStamped()
                target_pose_base_stamped.header.frame_id = self.base_frame
                target_pose_base_stamped.header.stamp = self.get_clock().now().to_msg()
                target_pose_base_stamped.pose = target_pose_base

            except Exception as e:
                self.get_logger().error(f"Transform Error: {e}")
                time.sleep(0.5)
                continue

            # 3. Calculate Error (Distance from Current TCP to Target)
            try:
                t_base_tcp = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
                curr_x = t_base_tcp.transform.translation.x
                curr_y = t_base_tcp.transform.translation.y
                curr_z = t_base_tcp.transform.translation.z
                
                tgt_x = target_pose_base.position.x
                tgt_y = target_pose_base.position.y
                tgt_z = target_pose_base.position.z
                
                dist_error = math.sqrt((tgt_x-curr_x)**2 + (tgt_y-curr_y)**2 + (tgt_z-curr_z)**2)
                
                # Feedback
                feedback = VisualServo.Feedback()
                feedback.current_error = dist_error
                goal_handle.publish_feedback(feedback)
                
                self.get_logger().info(f"   Loop {attempt+1}: Error = {dist_error:.4f} m")
                
                # Check Success
                if dist_error < tolerance:
                    self.get_logger().info("✅ Target Aligned!")
                    result.success = True
                    result.final_error = dist_error
                    goal_handle.succeed()
                    return result
                    
            except Exception:
                pass

            # 4. Move to Target (Cartesian Path)
            success = await self.move_to_pose(target_pose_base_stamped, goal_handle)
            
            if not success:
                self.get_logger().warn("⚠️ Movement failed, retrying...")
            
            time.sleep(0.5)

        self.get_logger().warn("❌ Max retries reached.")
        result.success = False
        result.final_error = dist_error
        goal_handle.abort()
        return result

    async def move_to_pose(self, pose_stamped, goal_handle_server):
        """
        เคลื่อนที่ไปหา Pose เป้าหมายด้วย Cartesian Path
        """
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = [pose_stamped.pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = True

        future_path = self._cartesian_client.call_async(req)
        
        while not future_path.done():
            if goal_handle_server.is_cancel_requested: return False
            time.sleep(0.01)
        
        response = future_path.result()

        if response.error_code.val != 1 or response.fraction < 0.5: # ยอมรับ Fraction ต่ำหน่อยเผื่อติด Singularity
            self.get_logger().warn(f"Path planning incomplete! Fraction: {response.fraction:.2f}")
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        future_exec = self._execute_client.send_goal_async(goal_msg)
        
        while not future_exec.done():
            if goal_handle_server.is_cancel_requested: return False
            time.sleep(0.01)
            
        goal_handle_exec = future_exec.result()
        
        if not goal_handle_exec.accepted:
            return False

        future_result = goal_handle_exec.get_result_async()
        
        while not future_result.done():
            if goal_handle_server.is_cancel_requested:
                goal_handle_exec.cancel_goal_async()
                return False
            time.sleep(0.1)
            
        return future_result.result().result.error_code.val == 1

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()