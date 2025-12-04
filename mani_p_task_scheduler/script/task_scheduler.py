#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand
from tf2_ros import Buffer, TransformListener
import sys
import math
import time
from enum import Enum, auto

class State(Enum):
    INIT = auto()
    SCAN = auto()
    APPROACH = auto()
    ALIGN = auto()
    GRASP_APPROACH = auto()
    GRASP_ACTION = auto()
    RETREAT = auto()
    DONE = auto()
    FAIL = auto()

class PickAndPlaceScheduler(Node):

    def __init__(self):
        super().__init__('task_scheduler')
        
        # --- ⚙️ CONFIG ⚙️ ---
        self.arm_group_name = "arm"      
        self.ee_link = "tcp_link"        
        self.base_frame = "Base_link"
        self.target_tag = "tag2"         # Default Target
        
        # Parameters
        self.approach_distance = 0.25    # 25cm standoff
        self.grasp_distance = 0.15       # 15cm push
        self.align_tolerance = 0.002     # 2mm alignment
        
        # --------------------

        # Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('🤖 Pick & Place Scheduler Ready.')

    def run(self):
        """ Main FSM Loop """
        state = State.INIT
        
        while rclpy.ok():
            if state == State.INIT:
                self.get_logger().info("🔵 STATE: INIT")
                # Wait for clients?
                if not self._move_group_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().error("MoveGroup Action not available!")
                    state = State.FAIL
                    continue
                state = State.SCAN

            elif state == State.SCAN:
                self.get_logger().info(f"🔵 STATE: SCAN (Looking for {self.target_tag})")
                # Check if we can see the tag
                if self.check_tf(self.target_tag):
                    self.get_logger().info("   ✅ Tag Found!")
                    state = State.APPROACH
                else:
                    self.get_logger().warn("   ⚠️ Tag not visible. Retrying...")
                    time.sleep(1.0)

            elif state == State.APPROACH:
                self.get_logger().info("🔵 STATE: APPROACH (Coarse)")
                success = self.go_to_tag_smart(self.target_tag, self.approach_distance)
                if success:
                    state = State.ALIGN
                else:
                    self.get_logger().error("   ❌ Approach Failed.")
                    state = State.FAIL

            elif state == State.ALIGN:
                self.get_logger().info("🔵 STATE: ALIGN (Visual Servo)")
                # Use Eye-in-Hand to align
                success = self.visual_servo_align(self.target_tag)
                if success:
                    state = State.GRASP_APPROACH
                else:
                    self.get_logger().warn("   ⚠️ Alignment incomplete, but proceeding...")
                    state = State.GRASP_APPROACH # Proceed anyway?

            elif state == State.GRASP_APPROACH:
                self.get_logger().info("🔵 STATE: GRASP APPROACH (Push)")
                self.control_gripper(open=True) # Open gripper first
                success = self.move_linear(self.grasp_distance)
                if success:
                    state = State.GRASP_ACTION
                else:
                    state = State.FAIL

            elif state == State.GRASP_ACTION:
                self.get_logger().info("🔵 STATE: GRASP ACTION")
                self.control_gripper(open=False) # Close gripper
                time.sleep(1.0) # Wait for grasp
                state = State.RETREAT

            elif state == State.RETREAT:
                self.get_logger().info("🔵 STATE: RETREAT")
                success = self.move_linear(-self.grasp_distance) # Pull back
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

    # --- ACTION IMPLEMENTATIONS ---

    def check_tf(self, target_frame):
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return True
        except:
            return False

    def go_to_tag_smart(self, tag_frame, distance):
        """ Logic from approach_tag_smart.py """
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

        # Position
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

        # Orientation
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = tag_frame
        ori_con.link_name = self.ee_link
        ori_con.absolute_x_axis_tolerance = 0.2
        ori_con.absolute_y_axis_tolerance = 0.2
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0
        ori_con.orientation.w = 1.0 # Identity (Face tag)

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
        """ Logic from visual_servo_align.py """
        for _ in range(5): # Max 5 attempts
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

            # Limit step
            max_step = 0.02
            if dist > max_step:
                scale = max_step / dist
                dx *= scale
                dy *= scale
            
            # Move relative
            self.move_relative(dx, dy, 0.0)
            time.sleep(0.5)
        
        return False # Timeout

    def move_relative(self, x, y, z):
        """ Move TCP relative to itself (Cartesian) """
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except:
            return False

        # Rotate vector to base frame
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
        """ Move forward/backward along Z axis """
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
    rclpy.init(args=args)
    
    # CLI Argument for Tag
    target_tag = "tag2"
    if len(sys.argv) > 1:
        target_tag = sys.argv[1]

    scheduler = PickAndPlaceScheduler()
    scheduler.target_tag = target_tag
    scheduler.run()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
