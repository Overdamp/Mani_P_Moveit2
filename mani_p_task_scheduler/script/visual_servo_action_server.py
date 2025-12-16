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
# MoveIt
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
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
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group) # PTP Client
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        # 3. TF Listener Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 4. Debugging Tools
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        self.target_pub = self.create_publisher(PoseStamped, 'visual_servo_target', 10) # Debug Publisher

        self.get_logger().info('✅ Visual Servo Action Server (Hybrid PTP/Cartesian) Ready.')

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
        dist_error = 0.0 
        
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
                target_pose_tag = PoseStamped()
                target_pose_tag.header.frame_id = tag_frame
                target_pose_tag.header.stamp = t_base_tag.header.stamp
                target_pose_tag.pose.position.z = target_standoff 
                
                # Orientation: Rotate 180 around X to face the tag
                target_pose_tag.pose.orientation.x = 1.0
                target_pose_tag.pose.orientation.y = 0.0
                target_pose_tag.pose.orientation.z = 0.0
                target_pose_tag.pose.orientation.w = 0.0
                
                # Transform to Base
                target_pose_base = tf2_geometry_msgs.do_transform_pose(target_pose_tag.pose, t_base_tag)
                
                target_pose_base_stamped = PoseStamped()
                target_pose_base_stamped.header.frame_id = self.base_frame
                target_pose_base_stamped.header.stamp = self.get_clock().now().to_msg()
                target_pose_base_stamped.pose = target_pose_base
                
                # Publish for Debug
                self.target_pub.publish(target_pose_base_stamped)

            except Exception as e:
                self.get_logger().error(f"Transform Error: {e}")
                time.sleep(0.5)
                continue

            # 3. Calculate Error
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

            # 4. Move to Target (Hybrid Logic)
            success = False
            if dist_error > 0.10: # If error > 10cm, use PTP (MoveGroup)
                self.get_logger().info("   🚀 Large Error: Using MoveGroup (PTP)")
                success = await self.move_to_pose_ptp(target_pose_base_stamped, goal_handle)
            else: # If error is small, use Cartesian
                self.get_logger().info("   👌 Small Error: Using Cartesian Path")
                success = await self.move_to_pose_cartesian(target_pose_base_stamped, goal_handle)
            
            if not success:
                self.get_logger().warn("⚠️ Movement failed, retrying...")
            
            time.sleep(0.5)

        self.get_logger().warn("❌ Max retries reached.")
        result.success = False
        result.final_error = dist_error
        goal_handle.abort()
        return result

    async def move_to_pose_ptp(self, pose_stamped, goal_handle_server):
        """ Point-to-Point Movement using MoveGroup (Robust for large moves) """
        if not self._move_group_client.wait_for_server(timeout_sec=2.0):
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.arm_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Constraints
        pcm = PositionConstraint()
        pcm.header.frame_id = self.base_frame
        pcm.link_name = self.ee_link
        pcm.constraint_region.primitive_poses.append(pose_stamped.pose)
        from shape_msgs.msg import SolidPrimitive
        pcm.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
        pcm.weight = 1.0
        
        ocm = OrientationConstraint()
        ocm.header.frame_id = self.base_frame
        ocm.link_name = self.ee_link
        ocm.orientation = pose_stamped.pose.orientation
        ocm.absolute_x_axis_tolerance = 0.1
        ocm.absolute_y_axis_tolerance = 0.1
        ocm.absolute_z_axis_tolerance = 0.1
        ocm.weight = 1.0
        
        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
        
        future = self._move_group_client.send_goal_async(goal_msg)
        while not future.done():
            if goal_handle_server.is_cancel_requested: return False
            time.sleep(0.1)
            
        goal_handle_moveit = future.result()
        if not goal_handle_moveit.accepted: return False
        
        res_future = goal_handle_moveit.get_result_async()
        while not res_future.done():
            if goal_handle_server.is_cancel_requested:
                goal_handle_moveit.cancel_goal_async()
                return False
            time.sleep(0.1)
            
        return res_future.result().result.error_code.val == 1

    async def move_to_pose_cartesian(self, pose_stamped, goal_handle_server):
        """ Cartesian Linear Movement (Precise for small moves) """
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

        if response.error_code.val != 1 or response.fraction < 0.5:
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