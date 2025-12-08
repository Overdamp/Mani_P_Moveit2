#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint
from moveit_msgs.srv import GetCartesianPath
from tf2_ros import Buffer, TransformListener
import sys
import math
import time

class LevelAdjuster(Node):
    def __init__(self):
        super().__init__('level_adjuster')
        
        self.arm_group_name = "arm"
        self.ee_link = "tcp_link"
        self.base_frame = "Base_link"
        
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info('Level Adjuster Ready.')

    def euler_from_quaternion(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def adjust_orientation(self, target_roll_deg=0.0):
        # 1. Get Current Pose
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")
            return

        current_x = t.transform.translation.x
        current_y = t.transform.translation.y
        current_z = t.transform.translation.z
        
        # 2. Extract Yaw
        _, _, current_yaw = self.euler_from_quaternion(t.transform.rotation)
        
        # 3. Create New Orientation (Align with ZED Camera)
        # User Request: orientation of tcp_link = zed_left_camera_optical_frame
        
        target_frame = "zed_mani_left_camera_optical_frame"
        try:
            t_zed = self.tf_buffer.lookup_transform(
                self.base_frame, target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not lookup {target_frame}: {e}")
            return

        q_new = [
            t_zed.transform.rotation.x,
            t_zed.transform.rotation.y,
            t_zed.transform.rotation.z,
            t_zed.transform.rotation.w
        ]
        
        self.get_logger().info(f"🔄 Adjusting to match {target_frame}")

        # 4. Cartesian Path (Task Space)
        # User requested "task_space" mode to force alignment.
        
        target_pose = Pose()
        target_pose.position.x = current_x
        target_pose.position.y = current_y
        target_pose.position.z = current_z
        target_pose.orientation.x = q_new[0]
        target_pose.orientation.y = q_new[1]
        target_pose.orientation.z = q_new[2]
        target_pose.orientation.w = q_new[3]
        
        waypoints = [target_pose]
        
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = 0.01 # 1cm resolution
        req.jump_threshold = 0.0 # Disable jump check for rotation
        req.avoid_collisions = True
        
        self.get_logger().info("🔄 Computing Cartesian Path...")
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        
        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")
            return

        # Execute
        from moveit_msgs.action import ExecuteTrajectory
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        self._execute_client.wait_for_server()
        send_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected!')
            return

        self.get_logger().info('⏳ Executing Cartesian Move...')
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('✅ Adjustment Complete!')
        else:
            self.get_logger().error(f'❌ Failed: {result.error_code.val}')

        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected!')
            return

        self.get_logger().info('⏳ Adjusting...')
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('✅ Adjustment Complete!')
        else:
            self.get_logger().error(f'❌ Failed: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    
    roll_deg = 0.0
    if len(sys.argv) > 1:
        roll_deg = float(sys.argv[1])
        
    node = LevelAdjuster()
    # Spin once to get TF
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        
    node.adjust_orientation(roll_deg)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
