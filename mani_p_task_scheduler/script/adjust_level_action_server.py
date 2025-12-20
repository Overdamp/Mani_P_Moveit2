#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionServer, ActionClient  # นำเข้า ActionServer และ ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup  # นำเข้า Callback Group แบบ Reentrant
from geometry_msgs.msg import Pose  # นำเข้า message types
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition
from moveit_msgs.action import ExecuteTrajectory  # นำเข้า action definition
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import math  # นำเข้า math
import time  # นำเข้า time

# Custom Action Interface (นำเข้า Action Interface ที่สร้างเอง)
from mani_p_actions.action import AdjustLevel
from sensor_msgs.msg import JointState  # นำเข้า message types สำหรับ Joint State

class AdjustLevelActionServer(Node):

    def __init__(self):
        super().__init__('adjust_level_action_server')  # สร้าง Node ชื่อ 'adjust_level_action_server'
        
        self.arm_group_name = "arm"      # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"        # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"    # ชื่อ frame อ้างอิง
        
        self.cb_group = ReentrantCallbackGroup()  # สร้าง Callback Group เพื่อให้ทำงานขนานกันได้

        # สร้าง Action Server สำหรับ AdjustLevel
        self._action_server = ActionServer(
            self,
            AdjustLevel,
            'adjust_level',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # สร้าง Action Client สำหรับ ExecuteTrajectory และ Service Client สำหรับ GetCartesianPath
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('✅ Adjust Level Action Server Ready.')
        
        # Joint State Subscription (รับค่า Joint State)
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        # Callback สำหรับเก็บค่า Joint ปัจจุบัน
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def log_status(self):
        # ฟังก์ชันสำหรับ Log สถานะ TCP และ Joint
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
        # Callback หลักเมื่อได้รับ Goal
        self.get_logger().info("🔄 Adjusting Level (Align with ZED)...")
        
        # 1. Get Current Pose (ดึงตำแหน่งปัจจุบัน)
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
        
        # 2. Get Target Orientation (ZED Camera) (ดึงค่าการหมุนเป้าหมายจากกล้อง ZED)
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

        # 3. Cartesian Path (วางแผนเส้นทางแบบ Cartesian)
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
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return AdjustLevel.Result(success=False)
            time.sleep(0.01)
        response = future.result()
        
        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

        # Execute (สั่งเคลื่อนที่)
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_future = self._execute_client.send_goal_async(goal_msg)
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return AdjustLevel.Result(success=False)
            time.sleep(0.01)
        goal_handle_exec = send_future.result()
        
        if not goal_handle_exec.accepted:
            goal_handle.abort()
            return AdjustLevel.Result(success=False)

        res_future = goal_handle_exec.get_result_async()
        while not res_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancellation requested. Cancelling trajectory...')
                cancel_future = goal_handle_exec.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.01)
                goal_handle.canceled()
                return AdjustLevel.Result(success=False)
            
            self.log_status()
            time.sleep(0.5)
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
