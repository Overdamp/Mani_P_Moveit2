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
from mani_p_actions.action import AdjustLevel

class AdjustLevelActionServer(Node):

    def __init__(self):
        super().__init__('adjust_level_action_server')
        
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            AdjustLevel,
            'adjust_level',
            self.execute_callback,
            callback_group=self.cb_group
        )

        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('✅ Adjust Level Action Server Ready.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("🔄 Adjusting Level (Align with ZED)...")
        
        # 1. Get Current Pose
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

        current_x = t.transform.translation.x
        current_y = t.transform.translation.y
        current_z = t.transform.translation.z
        
        # 2. Get Target Orientation (ZED Camera)
        target_frame = "zed_left_camera_optical_frame"
        try:
            t_zed = self.tf_buffer.lookup_transform(
                self.base_frame, target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not lookup {target_frame}: {e}")
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

        q_new = [
            t_zed.transform.rotation.x,
            t_zed.transform.rotation.y,
            t_zed.transform.rotation.z,
            t_zed.transform.rotation.w
        ]

        # 3. Cartesian Path
        target_pose = Pose()
        target_pose.position.x = current_x
        target_pose.position.y = current_y
        target_pose.position.z = current_z
        target_pose.orientation.x = q_new[0]
        target_pose.orientation.y = q_new[1]
        target_pose.orientation.z = q_new[2]
        target_pose.orientation.w = q_new[3]
        
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
            time.sleep(0.01)
        response = future.result()
        
        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

        # Execute
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_future = self._execute_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)
        goal_handle_exec = send_future.result()
        
        if not goal_handle_exec.accepted:
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

        res_future = goal_handle_exec.get_result_async()
        while not res_future.done():
            time.sleep(0.01)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            goal_handle.succeed()
            return AdjustLevel.Result(success=True)
        else:
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    node = AdjustLevelActionServer()
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
