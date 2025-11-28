#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener
import sys
import math
import time

class VisualServoAligner(Node):

    def __init__(self):
        super().__init__('visual_servo_aligner')
        
        # --- ⚙️ CONFIG ⚙️ ---
        self.arm_group_name = "arm"      
        self.ee_link = "tcp_link"        
        self.base_frame = "Base_link"    
        
        # Tolerance (meters)
        self.tolerance = 0.002 # 2mm
        self.max_step = 0.02   # Max movement per step (2cm)
        self.max_retries = 5   # Max attempts to align
        # --------------------

        # Action Client for Trajectory Execution
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        # Service Client for Cartesian Path
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Visual Servo Aligner Ready.')

    def align_to_tag(self, tag_frame):
        """ Iteratively align tcp_link to tag_frame in X/Y axes """
        self.get_logger().info(f"🎯 Aligning to {tag_frame}...")

        for attempt in range(self.max_retries):
            # 1. Get Error (Tag position in TCP frame)
            try:
                # Look up Tag in TCP frame
                # If Tag is at (0.01, -0.01, 0.2) in TCP frame, it means TCP needs to move (+0.01, -0.01) to align.
                t = self.tf_buffer.lookup_transform(
                    self.ee_link,
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().warn(f"⚠️ Could not see {tag_frame}. Is it in view?")
                return

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist_error = math.sqrt(dx*dx + dy*dy)

            self.get_logger().info(f"   Attempt {attempt+1}: Error X={dx:.4f}, Y={dy:.4f} (Dist={dist_error:.4f})")

            # 2. Check Tolerance
            if dist_error < self.tolerance:
                self.get_logger().info("✅ Aligned! Error is within tolerance.")
                return

            # 3. Limit Step Size (Safety)
            step_x = dx
            step_y = dy
            
            # Scale down if too large
            if dist_error > self.max_step:
                scale = self.max_step / dist_error
                step_x *= scale
                step_y *= scale
                self.get_logger().info(f"   ⚠️ Error too large, scaling step to {self.max_step}m")

            # 4. Move Relative to TCP (Cartesian)
            self.move_relative(step_x, step_y, 0.0)
            
            # Wait a bit for robot to settle
            time.sleep(1.0)

        self.get_logger().warn("❌ Max retries reached. Alignment might not be perfect.")

    def move_relative(self, x, y, z):
        """ Move tcp_link relative to itself """
        
        # 1. Get Current Pose in Base Frame
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")
            return

        # 2. Calculate Target Pose
        # We want to move by (x,y,z) in TCP frame.
        # Need to rotate this vector into Base frame.
        
        # Quaternion (Base -> TCP)
        qx = t_base.transform.rotation.x
        qy = t_base.transform.rotation.y
        qz = t_base.transform.rotation.z
        qw = t_base.transform.rotation.w
        
        # Rotate vector (x,y,z) by quaternion q
        # v_rotated = q * v * q_inv
        # ... Implementing standard quaternion rotation ...
        
        # Helper for rotation
        # x_new = (1-2yy-2zz)x + (2xy-2zw)y + (2xz+2yw)z
        # y_new = (2xy+2zw)x + (1-2xx-2zz)y + (2yz-2xw)z
        # z_new = (2xz-2yw)x + (2yz+2xw)y + (1-2xx-2yy)z
        
        vx, vy, vz = x, y, z
        
        rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qz*qw)*vy + (2*qx*qz + 2*qy*qw)*vz
        ry = (2*qx*qy + 2*qz*qw)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qx*qw)*vz
        rz = (2*qx*qz - 2*qy*qw)*vx + (2*qy*qz + 2*qx*qw)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz
        
        target_pose = Pose()
        target_pose.position.x = t_base.transform.translation.x + rx
        target_pose.position.y = t_base.transform.translation.y + ry
        target_pose.position.z = t_base.transform.translation.z + rz
        target_pose.orientation = t_base.transform.rotation # Keep same orientation

        # 3. Request Cartesian Path
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
            self.get_logger().error(f"❌ Path Planning Failed: {response.error_code.val}")
            return

        # 4. Execute
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        self._execute_client.wait_for_server()
        send_goal_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Execution Rejected!')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("\n⚠️  Usage Error!")
        print("   Syntax: ros2 run <pkg> visual_servo_align.py <TAG_NAME>")
        print("   Example: ros2 run mani_p_moveit_config3 visual_servo_align.py tag2\n")
        return

    tag_name = sys.argv[1]

    aligner = VisualServoAligner()
    aligner.align_to_tag(tag_name)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
