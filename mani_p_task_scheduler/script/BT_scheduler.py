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

# ==========================================
# 🌳 LIGHTWEIGHT BEHAVIOR TREE FRAMEWORK 🌳
# ==========================================

class NodeStatus(Enum):
    SUCCESS = auto()
    FAILURE = auto()
    RUNNING = auto()

class TreeNode:
    def __init__(self, name):
        self.name = name
        self.status = NodeStatus.FAILURE

    def tick(self):
        raise NotImplementedError("Tick not implemented")

class Sequence(TreeNode):
    """ Runs children sequentially. Fails if any child fails. """
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for child in self.children:
            result = child.tick()
            if result != NodeStatus.SUCCESS:
                self.status = result
                return result
        self.status = NodeStatus.SUCCESS
        return NodeStatus.SUCCESS

class Selector(TreeNode):
    """ Runs children sequentially. Succeeds if any child succeeds. """
    def __init__(self, name, children):
        super().__init__(name)
        self.children = children

    def tick(self):
        for child in self.children:
            result = child.tick()
            if result == NodeStatus.SUCCESS:
                self.status = NodeStatus.SUCCESS
                return NodeStatus.SUCCESS
            if result == NodeStatus.RUNNING:
                self.status = NodeStatus.RUNNING
                return NodeStatus.RUNNING
        self.status = NodeStatus.FAILURE
        return NodeStatus.FAILURE

class Action(TreeNode):
    """ Leaf node that performs a task. """
    def __init__(self, name, action_func):
        super().__init__(name)
        self.action_func = action_func

    def tick(self):
        # print(f"   [Action] {self.name}...")
        if self.action_func():
            self.status = NodeStatus.SUCCESS
            return NodeStatus.SUCCESS
        else:
            self.status = NodeStatus.FAILURE
            return NodeStatus.FAILURE

# ==========================================
# 🤖 ROBOT BEHAVIOR NODE 🤖
# ==========================================

class PickAndPlaceBT(Node):

    def __init__(self):
        super().__init__('bt_scheduler')
        
        # --- CONFIG ---
        self.arm_group_name = "arm"      
        self.ee_link = "tcp_link"        
        self.base_frame = "Base_link"
        self.target_tag = "tag2"
        
        self.approach_dist = 0.25
        self.grasp_dist = 0.15
        self.align_tol = 0.002
        # --------------

        # Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action')
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self._gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_cmd')

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('🌳 BT Scheduler Ready.')

    # --- ACTION FUNCTIONS (Return True/False) ---

    def check_system(self):
        if not self._move_group_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Waiting for MoveGroup...")
            return False
        return True

    def find_tag(self):
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.target_tag, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
            self.get_logger().info(f"✅ Found {self.target_tag}")
            return True
        except:
            self.get_logger().warn(f"Searching for {self.target_tag}...")
            return False

    def approach_tag(self):
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
        self.get_logger().info("👐 Opening Gripper")
        return self.control_gripper(open=True)

    def close_gripper(self):
        self.get_logger().info("✊ Closing Gripper")
        return self.control_gripper(open=False)

    def push_forward(self):
        self.get_logger().info("⬇️ Pushing Forward")
        return self.move_linear(self.grasp_dist)

    def pull_back(self):
        self.get_logger().info("⬆️ Pulling Back")
        return self.move_linear(-self.grasp_dist)

    # --- HELPERS ---

    def move_relative(self, x, y, z):
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
        goal = GripperCommand.Goal()
        goal.command.position = 0.03 if open else -0.01
        goal.command.max_effort = 100.0
        future = self._gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        return True

def main(args=None):
    rclpy.init(args=args)
    
    # CLI Argument
    target_tag = "tag2"
    if len(sys.argv) > 1:
        target_tag = sys.argv[1]

    # Create Node
    robot = PickAndPlaceBT()
    robot.target_tag = target_tag

    # ==========================
    # 🌳 BUILD THE TREE 🌳
    # ==========================
    
    # 1. Grasp Sequence (Push -> Close -> Pull)
    grasp_seq = Sequence("GraspSequence", [
        Action("OpenGripper", robot.open_gripper),
        Action("PushForward", robot.push_forward),
        Action("CloseGripper", robot.close_gripper),
        Action("PullBack", robot.pull_back)
    ])

    # 2. Main Sequence
    root = Sequence("MainTask", [
        Action("CheckSystem", robot.check_system),
        Action("FindTag", robot.find_tag),
        Action("ApproachTag", robot.approach_tag),
        Action("VisualServo", robot.visual_servo),
        grasp_seq
    ])

    # ==========================
    # 🏃 RUN THE TREE 🏃
    # ==========================
    
    robot.get_logger().info("--- STARTING BEHAVIOR TREE ---")
    
    # Simple Loop (Tick until Success or Failure)
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
