#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import Constraints, OrientationConstraint
from tf2_ros import Buffer, TransformListener
import sys
import math
import argparse
import tf_transformations
import numpy

class TCPAdjuster(Node):
    def __init__(self):
        super().__init__('tcp_adjuster')
        
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('TCP Adjuster Ready.')

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def move_to_tag(self, tag_id, offset):
        tag_frame = f"tag{tag_id}_fisheye_level"
        
        self.get_logger().info(f"🔍 Looking for {tag_frame}...")
        
        # Wait for TF (Spinning to ensure buffer fills)
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_frame, rclpy.time.Time())
                break # Found!
            except Exception:
                if (self.get_clock().now() - start_time).nanoseconds > 5e9: # 5 seconds
                    self.get_logger().error(f"❌ Timeout waiting for {tag_frame}")
                    return
                pass

        # Tag Pose
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        tr = t.transform.rotation
        
        # Convert Tag Quaternion to Matrix
        tag_q = [tr.x, tr.y, tr.z, tr.w]
        tag_matrix = tf_transformations.quaternion_matrix(tag_q)
        
        # Apply Offset along Tag's Z-axis (Blue arrow coming out of tag)
        # We want to be 'offset' meters away from the tag face
        target_x = tx + (tag_matrix[0, 2] * offset)
        target_y = ty + (tag_matrix[1, 2] * offset)
        target_z = tz + (tag_matrix[2, 2] * offset)
        
        # User requested to be higher from the shelf (Vertical Z offset)
        VERTICAL_OFFSET = 0.025
        target_z += VERTICAL_OFFSET
        
        # Orientation Logic:
        # We want the TCP to point AT the tag (Z-axis opposes Tag Z-axis)
        # BUT we want to keep the TCP's "UP" vector aligned with the World's "UP" (or consistent)
        # so that the gripper doesn't flip upside down when the tag is rotated.
        
        # 1. Calculate Vector from TCP to Tag (Direction Vector)
        # Since we want to align with Tag's Z axis (which points OUT), 
        # the Approach Vector is -Tag_Z (pointing INTO the tag).
        # Actually, simpler: We want TCP Z to be parallel to Tag Z but opposite direction.
        
        # Tag Z axis in World Frame
        tag_z_axis = tag_matrix[:3, 2] # [x, y, z]
        
        # Target TCP Z axis = -Tag Z axis (Face to Face)
        target_tcp_z = -tag_z_axis
        
        # Target TCP X axis (The "Up" or "Side" of the gripper)
        # We want to force this to be horizontal or aligned with World Z?
        # Usually Gripper X or Y is the "width". Let's assume we want TCP X to be parallel to World XY plane.
        # Or simpler: Just take the Tag's Yaw, and construct a pure Yaw rotation, then pitch down?
        
        # Let's try: Use Tag's Position, but Fixed Orientation relative to Base?
        # No, we need to face the tag.
        
        # Better approach: Look at the Tag, but keep "Up" vector fixed.
        # Calculate Yaw from Tag Position relative to Robot? 
        # No, Tag might be on the side.
        
        # Let's use the Tag's Yaw only, and ignore Roll/Pitch of the tag?
        # Extract Yaw from Tag Quaternion
        import math
        (tag_roll, tag_pitch, tag_yaw) = tf_transformations.euler_from_quaternion(tag_q)
        
        # We want the robot to face the tag.
        # If Tag is on a vertical wall, Tag Roll/Pitch might be relevant.
        # But user says "Cube upside down" -> Tag is likely rotated 180 roll.
        
        # Force Tag Roll = 0, Tag Pitch = 0 (Assume Tag is vertical?)
        # Or if Tag is on top of cube?
        
        # Let's assume we just want to match the Tag's Yaw, but keep TCP level.
        # But if the camera is looking down, we need to pitch.
        
        # User Request: "Don't flip gripper".
        # This usually means: Keep TCP Roll = 0 (relative to world).
        
        # New Strategy:
        # 1. Get Tag Rotation Matrix
        # 2. Extract Z-axis (Normal vector)
        # 3. Construct a new Rotation Matrix where:
        #    - Z-axis = Tag Z-axis (or opposite)
        #    - X-axis = Cross Product of (World Z, Z-axis) -> Horizontal
        #    - Y-axis = Cross Product of (Z-axis, X-axis)
        
        # Target Z (TCP Z) points INTO the tag (opposite to Tag Z)
        # Wait, standard TCP: Z points OUT of gripper.
        # So TCP Z should oppose Tag Z.
        tcp_z_vec = -tag_z_axis
        
        # World Up
        world_z = [0, 0, 1]
        
        # TCP Y (or X) should be perpendicular to TCP Z and World Z
        # This keeps the gripper "horizontal" relative to the ground
        tcp_y_vec = numpy.cross(tcp_z_vec, world_z)
        norm_y = numpy.linalg.norm(tcp_y_vec)
        
        if norm_y < 0.001:
            # Singularity: TCP Z is parallel to World Z (Looking straight up/down)
            # Just keep Tag's X axis or something default
            tcp_y_vec = [0, 1, 0] # Default
        else:
            tcp_y_vec = tcp_y_vec / norm_y
            
        # TCP X = Cross(Y, Z)
        tcp_x_vec = numpy.cross(tcp_y_vec, tcp_z_vec)
        tcp_x_vec = tcp_x_vec / numpy.linalg.norm(tcp_x_vec)
        
        # Construct Rotation Matrix
        R = numpy.identity(4)
        R[0, 0] = tcp_x_vec[0]; R[0, 1] = tcp_y_vec[0]; R[0, 2] = tcp_z_vec[0]
        R[1, 0] = tcp_x_vec[1]; R[1, 1] = tcp_y_vec[1]; R[1, 2] = tcp_z_vec[1]
        R[2, 0] = tcp_x_vec[2]; R[2, 1] = tcp_y_vec[2]; R[2, 2] = tcp_z_vec[2]
        
        # Convert to Quaternion
        target_q_np = tf_transformations.quaternion_from_matrix(R)
        
        # User Request: Pitch down 10 degrees
        # NOTE: Previous attempts (+5 and -5) both seemed to tilt up.
        # Let's try +10.0 to see the effect clearly.
        pitch_offset_rad = math.radians(10.0) 
        q_pitch = tf_transformations.quaternion_from_euler(0, pitch_offset_rad, 0)
        
        # Apply pitch: q_new = q_old * q_pitch
        target_q_np = tf_transformations.quaternion_multiply(target_q_np, q_pitch)
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z
        target_pose.pose.orientation.x = target_q_np[0]
        target_pose.pose.orientation.y = target_q_np[1]
        target_pose.pose.orientation.z = target_q_np[2]
        target_pose.pose.orientation.w = target_q_np[3]
        
        self.get_logger().info(f"🎯 Target: {tag_frame} + {offset}m (+{VERTICAL_OFFSET}m height)")
        self.get_logger().info(f"   Pos: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        self.get_logger().info("   Orientation: Level Horizon + 10deg Pitch")
        
        # ---------------------------------------------------------
        # Cartesian Path Logic (Linear Movement)
        # ---------------------------------------------------------
        
        # 1. Create Client
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Service 'compute_cartesian_path' not available")
            return

        # 2. Prepare Waypoints
        # We only have one waypoint (the target)
        # Note: In Cartesian path, we usually want to interpolate, but MoveIt does that for us
        # if we give it the end point. However, providing current pose as start is good practice?
        # Actually, waypoints list should just contain the target points.
        
        # Target Pose (Geometry Msg)
        target_pose_msg = target_pose.pose
        waypoints = [target_pose_msg]

        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = 0.01       # 1cm resolution
        req.jump_threshold = 0.0  # Disable jump check (or set to e.g. 2.0)
        req.avoid_collisions = True

        self.get_logger().info("📏 Computing Cartesian Path...")
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")
            # Fallback or Exit? Let's exit to be safe for fine tuning.
            return
            
        fraction = response.fraction
        self.get_logger().info(f"   Path Fraction: {fraction * 100:.1f}%")
        
        if fraction < 0.95:
             self.get_logger().warn(f"⚠️ Cartesian Path incomplete ({fraction*100:.1f}%)! Falling back to PTP...")
             
             # Fallback: Use standard MoveGroup (PTP)
             # This might not be a straight line, but will get there.
             
             # Construct Constraints for PTP
             constraints = Constraints()
             from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
             from shape_msgs.msg import SolidPrimitive
             
             # 1. Position
             pc = PositionConstraint()
             pc.header.frame_id = self.base_frame
             pc.link_name = self.ee_link
             pc.target_point_offset.x = 0.0
             pc.target_point_offset.y = 0.0
             pc.target_point_offset.z = 0.0
             
             box = SolidPrimitive()
             box.type = SolidPrimitive.BOX
             box.dimensions = [0.001, 0.001, 0.001] 
             pc.constraint_region.primitives.append(box)
             pc.constraint_region.primitive_poses.append(target_pose.pose)
             pc.weight = 1.0
             
             # 2. Orientation
             oc = OrientationConstraint()
             oc.header.frame_id = self.base_frame
             oc.link_name = self.ee_link
             oc.orientation = target_pose.pose.orientation
             oc.absolute_x_axis_tolerance = 0.01
             oc.absolute_y_axis_tolerance = 0.01
             oc.absolute_z_axis_tolerance = 0.01
             oc.weight = 1.0
             
             constraints.position_constraints.append(pc)
             constraints.orientation_constraints.append(oc)
             
             # Send Goal
             goal_msg = MoveGroup.Goal()
             goal_msg.request.group_name = self.arm_group_name
             goal_msg.request.num_planning_attempts = 10
             goal_msg.request.allowed_planning_time = 5.0
             goal_msg.request.max_velocity_scaling_factor = 0.1
             goal_msg.request.max_acceleration_scaling_factor = 0.1
             goal_msg.request.goal_constraints.append(constraints)
             
             self.get_logger().info("🚀 Sending PTP Goal (Fallback)...")
             self._action_client.wait_for_server()
             future = self._action_client.send_goal_async(goal_msg)
             rclpy.spin_until_future_complete(self, future)
             goal_handle = future.result()
             
             if not goal_handle.accepted:
                 self.get_logger().error('❌ PTP Goal Rejected!')
                 return

             res_future = goal_handle.get_result_async()
             rclpy.spin_until_future_complete(self, res_future)
             result = res_future.result().result
             
             if result.error_code.val == 1:
                 self.get_logger().info('✅ Arrived (PTP Fallback)!')
             else:
                 self.get_logger().error(f'❌ PTP Failed: {result.error_code.val}')
             return

        # 3. Execute Cartesian
        from moveit_msgs.action import ExecuteTrajectory
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._execute_client.wait_for_server()
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        self.get_logger().info("🚀 Executing Linear Move...")
        send_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected!')
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('✅ Arrived (Linear)!')
        else:
            self.get_logger().error(f'❌ Failed: {result.error_code.val}')

def main():
    rclpy.init()
    
    parser = argparse.ArgumentParser(description="Move TCP to tag_fisheye_level")
    parser.add_argument("tag_id", type=int, help="Tag ID (1, 2, 3)")
    parser.add_argument("--offset", type=float, default=0.1, help="Distance from tag (meters). Default 0.1")
    
    # Filter ROS args
    args = parser.parse_args([arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')])
    
    node = TCPAdjuster()
    node.move_to_tag(args.tag_id, args.offset)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
