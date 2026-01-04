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
from mani_p_actions.action import MoveLinear
from sensor_msgs.msg import JointState  # นำเข้า message types สำหรับ Joint State

class MoveLinearActionServer(Node):

    def __init__(self):
        super().__init__('move_linear_action_server')  # สร้าง Node ชื่อ 'move_linear_action_server'
        
        self.arm_group_name = "arm"      # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"        # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"    # ชื่อ frame อ้างอิง
        
        self.cb_group = ReentrantCallbackGroup()  # สร้าง Callback Group เพื่อให้ทำงานขนานกันได้

        # สร้าง Action Server สำหรับ MoveLinear
        self._action_server = ActionServer(
            self,
            MoveLinear,
            'move_linear',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # สร้าง Action Client สำหรับ ExecuteTrajectory และ Service Client สำหรับ GetCartesianPath
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory', callback_group=self.cb_group)
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path', callback_group=self.cb_group)
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('✅ Move Linear Action Server Ready.')
        
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
        direction = goal_handle.request.direction.lower() # รับค่าทิศทาง (แปลงเป็นตัวพิมพ์เล็ก)
        total_distance = goal_handle.request.distance # รับค่าระยะทางรวม
        
        self.get_logger().info(f"📏 Moving '{direction}' by {total_distance}m (Incremental)")
        
        # Execute Single Step
        success = await self.execute_linear_step(direction, total_distance, goal_handle)
        
        if not success:
            self.get_logger().error(f"❌ Failed to move linear")
            goal_handle.abort()
            return MoveLinear.Result(success=False)
            
        goal_handle.succeed()
        return MoveLinear.Result(success=True)

    async def execute_linear_step(self, direction, distance, goal_handle):
        # ฟังก์ชันย่อยสำหรับเดิน 1 ก้าวเล็กๆ
        
        # 1. Get Current Pose (ดึงตำแหน่งปัจจุบัน)
        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")
            return False

        # 2. Calculate Target Pose (คำนวณตำแหน่งเป้าหมาย)
        dx, dy, dz = 0.0, 0.0, 0.0
        
        if direction == "forward":
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            dx = (2*qx*qz + 2*qy*qw) * distance
            dy = (2*qy*qz - 2*qx*qw) * distance
            dz = (1 - 2*qx*qx - 2*qy*qy) * distance
            
        elif direction == "backward":
            qx = t.transform.rotation.x
            qy = t.transform.rotation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            dist_neg = -distance
            dx = (2*qx*qz + 2*qy*qw) * dist_neg
            dy = (2*qy*qz - 2*qx*qw) * dist_neg
            dz = (1 - 2*qx*qx - 2*qy*qy) * dist_neg
            
        elif direction == "up":
            dz = distance
        elif direction == "down":
            dz = -distance
        elif direction == "left":
            dy = distance
        elif direction == "right":
            dy = -distance
        else:
            return False

        target_pose = Pose()
        target_pose.position.x = t.transform.translation.x + dx
        target_pose.position.y = t.transform.translation.y + dy
        target_pose.position.z = t.transform.translation.z + dz
        target_pose.orientation = t.transform.rotation 
        
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
        # Wait for result (รอผลลัพธ์แบบ Async ใน Loop)
        while not future.done():
             if goal_handle.is_cancel_requested: return False
             time.sleep(0.01)
             
        response = future.result()
        
        if response.error_code.val != 1:
            self.get_logger().error(f"Cartesian Planning Failed: {response.error_code.val}")
            return False
            
        # Log number of points
        num_points = len(response.solution.joint_trajectory.points)
        # self.get_logger().info(f"      📝 Planned Path with {num_points} points")

        # 4. Execute
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        send_future = self._execute_client.send_goal_async(goal_msg)
        while not send_future.done():
             if goal_handle.is_cancel_requested: return False
             time.sleep(0.01)
             
        goal_handle_exec = send_future.result()
        
        if not goal_handle_exec.accepted:
            self.get_logger().error("Goal rejected by MoveIt server")
            return False

        res_future = goal_handle_exec.get_result_async()
        while not res_future.done():
            if goal_handle.is_cancel_requested:
                cancel_future = goal_handle_exec.cancel_goal_async()
                while not cancel_future.done(): time.sleep(0.01)
                return False
            # self.log_status() # ไม่ต้อง Log ถี่เกินไป
            time.sleep(0.1)
            
        result = res_future.result().result
        if result.error_code.val != 1:
            self.get_logger().error(f"❌ Execution Failed with Error Code: {result.error_code.val}")
            
        return result.error_code.val == 1

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
