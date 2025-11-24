#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener
import sys

class CartesianPusher(Node):

    def __init__(self):
        super().__init__('cartesian_pusher')
        
        # --- ⚙️ CONFIG ⚙️ ---
        self.arm_group_name = "arm"      
        self.ee_link = "tcp_link"        
        self.base_frame = "Base_link"    
        # --------------------

        # Action Client for Trajectory Execution
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        # Service Client for Cartesian Path
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Cartesian Pusher Ready.')

    def move_linear(self, distance):
        """ Move gripper forward (Z-axis) in Cartesian space """
        self.get_logger().info(f"📏 Cartesian Move: {distance}m")

        # 1. Get Current Pose (with Retry)
        t = None
        for i in range(10):
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                break # Found!
            except Exception as e:
                self.get_logger().warn(f"⏳ Waiting for TF... ({i+1}/10)")
                rclpy.spin_once(self, timeout_sec=0.5) # Spin to let TF arrive
        
        if t is None:
            self.get_logger().error(f"❌ Could not get current pose after retries.")
            return

        # 2. Calculate Target Pose (Current + Offset in Local Z)
        # q_current
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
        target_pose.orientation = t.transform.rotation # Keep same orientation

        # 3. Request Cartesian Path
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = [target_pose]
        req.max_step = 0.01 # 1cm step
        req.jump_threshold = 0.0 # Disable jump check
        req.avoid_collisions = True

        self.get_logger().info("⏳ Computing Cartesian Path...")
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Path Planning Failed: {response.error_code.val}")
            return
            
        if response.fraction < 0.9:
            self.get_logger().warn(f"⚠️ Path truncated! Only computed {response.fraction*100:.1f}%")

        # 4. Execute Trajectory
        self.get_logger().info(f"🚀 Executing Path ({len(response.solution.joint_trajectory.points)} points)...")
        
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
        
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('✅ SUCCESS: Cartesian Move Complete!')
        else:
            self.get_logger().error(f'❌ Execution Failed: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("\n⚠️  Usage Error!")
        print("   Syntax: ros2 run <pkg> cartesian_push.py [DISTANCE]")
        print("   Example: ros2 run mani_p_moveit_config3 cartesian_push.py 0.15\n")
        return

    dist = 0.15
    try:
        dist = float(sys.argv[1])
    except ValueError:
        print("⚠️ Warning: Invalid distance format. Using default 0.15m")

    pusher = CartesianPusher()
    pusher.move_linear(dist)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
