#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import sys
import math
import time

class Coarse2FineTask(Node):

    def __init__(self):
        super().__init__('coarse2fine_task')
        
        # --- CONFIG ---
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        # Coarse Settings
        self.coarse_offset_z = 0.25  # 25cm
        
        # Fine Settings
        self.fine_tolerance = 0.002 # 2mm
        self.fine_max_step = 0.02   # 2cm
        self.fine_max_retries = 10
        # --------------

        # MoveIt Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Coarse-to-Fine Task Ready.')

    # =========================================================================
    # STEP 1: COARSE APPROACH (MoveIt Constraints)
    # =========================================================================
    def step_1_coarse_approach(self, tag_frame_id):
        self.get_logger().info(f"🚀 [STEP 1] Coarse Approach to {tag_frame_id} (Offset {self.coarse_offset_z}m)...")

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
        constraints.name = f"Approach_{tag_frame_id}"

        # Position Constraint (Sphere around target)
        pos_con = PositionConstraint()
        pos_con.header.frame_id = tag_frame_id
        pos_con.link_name = self.ee_link
        pos_con.weight = 1.0
        
        region = BoundingVolume()
        s = SolidPrimitive()
        s.type = SolidPrimitive.SPHERE
        s.dimensions = [0.005]
        region.primitives.append(s)
        
        target_pose = Pose()
        target_pose.position.z = self.coarse_offset_z # Offset from Tag
        target_pose.orientation.w = 1.0
        
        region.primitive_poses.append(target_pose)
        pos_con.constraint_region = region
        
        # Orientation Constraint (Align with Tag)
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = tag_frame_id
        ori_con.link_name = self.ee_link
        
        # We want the gripper to FACE the tag.
        # Based on approach_tag_smart.py logic:
        # q_x = 0.0, q_y = 1.0, q_z = 0.0, q_w = 0.0 (for 0 degree roll)
        # This rotates the base orientation (facing tag) correctly.
        
        ori_con.orientation.x = 0.0
        ori_con.orientation.y = 1.0
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        
        ori_con.absolute_x_axis_tolerance = 0.5 # Relaxed
        ori_con.absolute_y_axis_tolerance = 0.5 # Relaxed
        ori_con.absolute_z_axis_tolerance = 0.1 # Strict Roll
        ori_con.weight = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        self._move_group_client.wait_for_server()
        future = self._move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Coarse Goal Rejected!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('✅ Coarse Approach Complete.')
            return True
        else:
            self.get_logger().error(f'❌ Coarse Approach Failed: {result.error_code.val}')
            return False

    # =========================================================================
    # STEP 2: FINE ALIGNMENT (Visual Servoing)
    # =========================================================================
    def step_2_fine_alignment(self, tag_frame_id):
        self.get_logger().info(f"🎯 [STEP 2] Fine Alignment to {tag_frame_id}...")
        
        for attempt in range(self.fine_max_retries):
            # 1. Get Error
            try:
                t = self.tf_buffer.lookup_transform(
                    self.ee_link, tag_frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            except Exception:
                self.get_logger().warn(f"⚠️ Cannot see {tag_frame_id}")
                return False

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist_error = math.sqrt(dx*dx + dy*dy)
            
            self.get_logger().info(f"   Iter {attempt+1}: Error X={dx:.4f}, Y={dy:.4f} (Dist={dist_error:.4f})")

            if dist_error < self.fine_tolerance:
                self.get_logger().info("✅ Aligned!")
                return True

            # 2. Calculate Step
            step_x, step_y = dx, dy
            if dist_error > self.fine_max_step:
                scale = self.fine_max_step / dist_error
                step_x *= scale
                step_y *= scale

            # 3. Move
            if not self.move_relative(step_x, step_y, 0.0):
                return False
            
            time.sleep(0.5) # Wait for settle

        self.get_logger().warn("❌ Max retries reached for fine alignment.")
        return False

    def move_relative(self, x, y, z):
        # Get Current Pose
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception:
            return False

        # Rotate vector to Base Frame
        qx, qy, qz, qw = t_base.transform.rotation.x, t_base.transform.rotation.y, t_base.transform.rotation.z, t_base.transform.rotation.w
        vx, vy, vz = x, y, z
        rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qz*qw)*vy + (2*qx*qz + 2*qy*qw)*vz
        ry = (2*qx*qy + 2*qz*qw)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qx*qw)*vz
        rz = (2*qx*qz - 2*qy*qw)*vx + (2*qy*qz + 2*qx*qw)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz

        target_pose = Pose()
        target_pose.position.x = t_base.transform.translation.x + rx
        target_pose.position.y = t_base.transform.translation.y + ry
        target_pose.position.z = t_base.transform.translation.z + rz
        target_pose.orientation = t_base.transform.rotation

        # Plan & Execute
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
    # STEP 3: CONFIRMATION
    # =========================================================================
    def step_3_confirm(self):
        self.get_logger().info("⏳ Waiting for 5 seconds...")
        time.sleep(5.0)
        self.get_logger().info("\n" + "="*40)
        self.get_logger().info("   🤖 READY TO PICK! 📦   ")
        self.get_logger().info("="*40 + "\n")

    def run(self, tag_id):
        if self.step_1_coarse_approach(tag_id):
            time.sleep(1.0)
            if self.step_2_fine_alignment(tag_id):
                self.step_3_confirm()

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: ros2 run mani_p_moveit_config3 coarse2fine_task.py <TAG_ID>")
        return

    tag_id = sys.argv[1]
    node = Coarse2FineTask()
    node.run(tag_id)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
