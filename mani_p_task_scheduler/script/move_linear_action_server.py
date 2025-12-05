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
from mani_p_actions.action import MoveLinear

class MoveLinearActionServer(Node):

    def __init__(self):
        super().__init__('move_linear_action_server')
        
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        self.cb_group = ReentrantCallbackGroup()

        self._action_server = ActionServer(
            self,
            MoveLinear,
            'move_linear',
            self.execute_callback,
            callback_group=self.cb_group
        )

        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('✅ Move Linear Action Server Ready.')

    async def execute_callback(self, goal_handle):
        distance = goal_handle.request.distance
        self.get_logger().info(f"📏 Moving Linear: {distance}m")
        
        # 1. Get Current Pose
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")
            goal_handle.abort()
            return MoveLinear.Result(success=False)

        # 2. Calculate Target Pose (Relative to TCP Z)
        # We need to rotate the movement vector (0, 0, distance) from TCP frame to Base frame
        
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        
        # Vector in TCP frame: (0, 0, distance)
        vx, vy, vz = 0.0, 0.0, distance
        
        # Rotate vector by quaternion
        # x_new = (2xz + 2yw) * v_z
        # y_new = (2yz - 2xw) * v_z
        # z_new = (1 - 2xx - 2yy) * v_z
        # Since vx=vy=0, we simplify:
        
        dx = (2*qx*qz + 2*qy*qw) * vz
        dy = (2*qy*qz - 2*qx*qw) * vz
        dz = (1 - 2*qx*qx - 2*qy*qy) * vz
        
        target_pose = Pose()
        target_pose.position.x = t.transform.translation.x + dx
        target_pose.position.y = t.transform.translation.y + dy
        target_pose.position.z = t.transform.translation.z + dz
        target_pose.orientation = t.transform.rotation # Keep same orientation
        
        # 3. Compute Cartesian Path
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
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveLinear.Result(success=False)
            time.sleep(0.01)
        response = future.result()
        
        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")
            goal_handle.abort()
            return MoveLinear.Result(success=False)

        # 4. Execute
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_future = self._execute_client.send_goal_async(goal_msg)
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveLinear.Result(success=False)
            time.sleep(0.01)
        goal_handle_exec = send_future.result()
        
        if not goal_handle_exec.accepted:
            goal_handle.abort()
            return MoveLinear.Result(success=False)

        res_future = goal_handle_exec.get_result_async()
        while not res_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancellation requested. Cancelling trajectory...')
                cancel_future = goal_handle_exec.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.01)
                goal_handle.canceled()
                return MoveLinear.Result(success=False)
            time.sleep(0.01)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            goal_handle.succeed()
            return MoveLinear.Result(success=True)
        else:
            goal_handle.abort()
            return MoveLinear.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    node = MoveLinearActionServer()
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
