#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener
import math
import time

# Custom Action Interface
# Custom Action Interface
from mani_p_actions.action import VisualServo
from sensor_msgs.msg import JointState

class VisualServoActionServer(Node):

    def __init__(self):
        super().__init__('visual_servo_action_server')
        
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            VisualServo,
            'visual_servo',
            self.execute_callback,
            callback_group=self.cb_group
        )

        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('✅ Visual Servo Action Server Ready.')
        
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
        tag_frame = goal_handle.request.tag_name
        tolerance = goal_handle.request.tolerance
        
        self.get_logger().info(f"🎯 Aligning to {tag_frame} (Tol: {tolerance}m)...")
        
        max_retries = 5
        max_step = 0.02
        
        for attempt in range(max_retries):
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return VisualServo.Result(success=False, final_error=0.0)

            # 1. Get Error
            try:
                t = self.tf_buffer.lookup_transform(
                    self.ee_link,
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().warn(f"⚠️ Could not see {tag_frame}")
                time.sleep(1.0)
                continue

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist_error = math.sqrt(dx*dx + dy*dy)
            
            feedback = VisualServo.Feedback()
            feedback.current_error = dist_error
            goal_handle.publish_feedback(feedback)
            
            self.get_logger().info(f"   Attempt {attempt+1}: Error={dist_error:.4f}")

            if dist_error < tolerance:
                goal_handle.succeed()
                return VisualServo.Result(success=True, final_error=dist_error)

            # Limit Step
            step_x = dx
            step_y = dy
            if dist_error > max_step:
                scale = max_step / dist_error
                step_x *= scale
                step_y *= scale

            # Move Relative
            success = self.move_relative(step_x, step_y, 0.0, goal_handle)
            if not success:
                self.get_logger().error("Failed to move relative")
                # Don't abort immediately, try again?
            
            time.sleep(1.0)

        goal_handle.abort()
        return VisualServo.Result(success=False, final_error=dist_error)

    def move_relative(self, x, y, z, goal_handle_server=None):
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except:
            return False

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
        while not future.done():
            if goal_handle_server and goal_handle_server.is_cancel_requested:
                return False
            time.sleep(0.01)
        response = future.result()

        if response.error_code.val != 1:
            return False

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_future = self._execute_client.send_goal_async(goal_msg)
        while not send_future.done():
            if goal_handle_server and goal_handle_server.is_cancel_requested:
                return False
            time.sleep(0.01)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            return False

        res_future = goal_handle.get_result_async()
        while not res_future.done():
            if goal_handle_server and goal_handle_server.is_cancel_requested:
                self.get_logger().info('Cancellation requested. Cancelling trajectory...')
                cancel_future = goal_handle.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.01)
                return False
            
            self.log_status()
            time.sleep(0.5)
            
        return res_future.result().result.error_code.val == 1

def main(args=None):
    rclpy.init(args=args)
    node = VisualServoActionServer()
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
