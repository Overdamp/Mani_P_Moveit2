#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import math
import time

# Custom Action Interface
# Custom Action Interface
from mani_p_actions.action import ApproachTag
from sensor_msgs.msg import JointState

class ApproachActionServer(Node):

    def __init__(self):
        super().__init__('approach_action_server')
        
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        # Callback Group for concurrency
        self.cb_group = ReentrantCallbackGroup()

        # Action Server
        self._action_server = ActionServer(
            self,
            ApproachTag,
            'approach_tag',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # MoveIt Clients
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('✅ Approach Action Server Ready.')
        
        # Joint State Subscription
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def log_status(self):
        try:
            if self.tf_buffer.can_transform(self.base_frame, self.ee_link, rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_link, rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                
                joints_str = ", ".join([f"{k}: {v:.3f}" for k, v in self.current_joints.items() if 'J' in k or 'palm' in k])
                print(f"📍 TCP: [{x:.3f}, {y:.3f}, {z:.3f}] | 🦾 Joints: {{{joints_str}}}")
        except Exception:
            pass

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"🚀 Executing Approach Goal: Tag={goal_handle.request.tag_name}, Dist={goal_handle.request.distance}")
        
        tag_frame = goal_handle.request.tag_name
        distance = goal_handle.request.distance
        roll_deg = goal_handle.request.roll_offset

        feedback_msg = ApproachTag.Feedback()
        feedback_msg.current_state = "Planning"
        goal_handle.publish_feedback(feedback_msg)

        # --- Logic from approach_tag_smart.py ---
        
        # 1. Create Goal
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

        # 2. Constraints
        constraints = Constraints()
        constraints.name = f"Approach_{tag_frame}"

        # Position Constraint
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
        
        # Orientation Constraint
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = tag_frame
        ori_con.link_name = self.ee_link
        
        half_angle = math.radians(roll_deg) / 2.0
        q_x = math.sin(half_angle)
        q_y = math.cos(half_angle)
        
        ori_con.orientation.x = q_x
        ori_con.orientation.y = q_y
        ori_con.orientation.z = 0.0
        ori_con.orientation.w = 0.0
        
        ori_con.absolute_x_axis_tolerance = 0.2
        ori_con.absolute_y_axis_tolerance = 0.2
        ori_con.absolute_z_axis_tolerance = 0.1
        ori_con.weight = 1.0

        constraints.position_constraints.append(pos_con)
        constraints.orientation_constraints.append(ori_con)
        goal_msg.request.goal_constraints.append(constraints)

        # 3. Send to MoveGroup
        feedback_msg.current_state = "Moving"
        goal_handle.publish_feedback(feedback_msg)
        
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup Action Server not available")
            goal_handle.abort()
            return ApproachTag.Result(success=False, message="MoveGroup unavailable")

        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance
        while not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled before acceptance')
                goal_handle.canceled()
                return ApproachTag.Result(success=False, message="Canceled")
            time.sleep(0.1)
        
        goal_handle_moveit = send_goal_future.result()
        
        if not goal_handle_moveit.accepted:
            self.get_logger().error('Goal Rejected by MoveIt')
            goal_handle.abort()
            return ApproachTag.Result(success=False, message="Goal Rejected")

        # Wait for result
        result_future = goal_handle_moveit.get_result_async()
        while not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancellation requested. Cancelling MoveIt goal...')
                cancel_future = goal_handle_moveit.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.01)
                goal_handle.canceled()
                return ApproachTag.Result(success=False, message="Canceled")
            
            self.log_status()
            time.sleep(0.5)
            
        result = result_future.result().result
        
        if result.error_code.val == 1:
            goal_handle.succeed()
            return ApproachTag.Result(success=True, message="Reached Tag")
        else:
            goal_handle.abort()
            return ApproachTag.Result(success=False, message=f"MoveIt Error: {result.error_code.val}")

def main(args=None):
    rclpy.init(args=args)
    node = ApproachActionServer()
    
    # Use MultiThreadedExecutor for ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
