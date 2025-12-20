#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.action import ActionServer  # นำเข้า ActionServer
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.callback_groups import ReentrantCallbackGroup  # นำเข้า Callback Group แบบ Reentrant
from rclpy.executors import MultiThreadedExecutor  # นำเข้า MultiThreadedExecutor

from mani_p_actions.action import MoveToNamedTarget  # นำเข้า Action Interface ที่สร้างเอง
from moveit_msgs.action import MoveGroup  # นำเข้า action definition สำหรับ MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint  # นำเข้า message types สำหรับข้อจำกัด
from rclpy.action import ActionClient  # นำเข้า ActionClient
from sensor_msgs.msg import JointState  # นำเข้า message types สำหรับ Joint State
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF

class MoveToNamedTargetActionServer(Node):
    def __init__(self):
        super().__init__('move_to_named_target_action_server')  # สร้าง Node ชื่อ 'move_to_named_target_action_server'
        
        self.callback_group = ReentrantCallbackGroup()  # สร้าง Callback Group เพื่อให้ทำงานขนานกันได้
        
        # สร้าง Action Server สำหรับ MoveToNamedTarget
        self._action_server = ActionServer(
            self,
            MoveToNamedTarget,
            'move_to_named_target',
            self.execute_callback,
            callback_group=self.callback_group)
            
        # MoveIt Action Client (สร้าง Client สำหรับ MoveGroup)
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.callback_group)
        
        self.get_logger().info("🏠 MoveToNamedTarget Action Server Ready")
        
        # TF Setup (ตั้งค่า TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
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
            if self.tf_buffer.can_transform('Base_link', 'tcp_link', rclpy.time.Time()):
                t = self.tf_buffer.lookup_transform('Base_link', 'tcp_link', rclpy.time.Time())
                x = t.transform.translation.x
                y = t.transform.translation.y
                z = t.transform.translation.z
                
                joints_str = ", ".join([f"{k}: {v:.3f}" for k, v in self.current_joints.items() if 'J' in k or 'palm' in k])
                print(f"📍 TCP: [{x:.3f}, {y:.3f}, {z:.3f}] | 🦾 Joints: {{{joints_str}}}")
        except Exception:
            pass

    async def execute_callback(self, goal_handle):
        # Callback หลักเมื่อได้รับ Goal
        target_name = goal_handle.request.target_name
        self.get_logger().info(f"Executing goal: Move to '{target_name}'")
        
        feedback_msg = MoveToNamedTarget.Feedback()
        result = MoveToNamedTarget.Result()
        
        feedback_msg.current_state = f"Planning for {target_name}..."
        goal_handle.publish_feedback(feedback_msg)
        
        # Execute Move (สั่งเคลื่อนที่)
        success = await self.move_to_named_target(target_name, goal_handle)
        
        if success:
            result.success = True
            result.message = f"Arrived at {target_name}."
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f"Failed to move to {target_name}."
            goal_handle.abort()
            
        return result

    async def move_to_named_target(self, target_name, goal_handle_server):
        # ฟังก์ชันสั่งเคลื่อนที่ไปยัง Named Target
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # กำหนดค่า Joint สำหรับ Named Target ต่างๆ
        
        joint_constraints = []
        
        if target_name.lower() == "home":
             # Define Home Pose (Standard OpenManipulator-P)
             # J1=0, J2=-1.0, J3=0.3, J4=0.7, J5=0, palm=0
             vals = {
                'J1': 0.0, 
                'J2': -1.0, 
                'J3': 0.3, 
                'J4': 0.7,
                'J5': 0.0,
                'palm_joint': 0.0
             }
             
        elif target_name.lower() == "sleep":
             # Folded Pose (ท่าพับเก็บ)
             vals = {
                'J1': 0.0, 
                'J2': -1.57, 
                'J3': 1.3, 
                'J4': 0.2,
                'J5': 0.0,
                'palm_joint': 0.0
             }
             
        else:
             self.get_logger().warn(f"Unknown named target: {target_name}. Using Home.")
             vals = {
                'J1': 0.0, 
                'J2': -1.0, 
                'J3': 0.3, 
                'J4': 0.7,
                'J5': 0.0,
                'palm_joint': 0.0
             }

        for joint, angle in vals.items():
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = angle
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)
            
        goal_msg.request.goal_constraints.append(Constraints(joint_constraints=joint_constraints))
        
        self.get_logger().info(f"🚀 Sending Goal to MoveIt for '{target_name}'")
        
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        
        # Wait for goal acceptance (รอการตอบรับ Goal)
        while not send_goal_future.done():
            if goal_handle_server.is_cancel_requested:
                self.get_logger().info('Goal canceled before acceptance')
                goal_handle_server.canceled()
                return False
            import time
            time.sleep(0.1)

        goal_handle_moveit = send_goal_future.result()
        
        if not goal_handle_moveit.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        result_future = goal_handle_moveit.get_result_async()
        
        # Wait for result with cancellation check (รอผลลัพธ์และตรวจสอบการยกเลิก)
        while not result_future.done():
            if goal_handle_server.is_cancel_requested:
                self.get_logger().info('Cancellation requested. Cancelling MoveIt goal...')
                cancel_future = goal_handle_moveit.cancel_goal_async()
                while not cancel_future.done():
                    import time
                    time.sleep(0.01)
                goal_handle_server.canceled()
                return False
            
            self.log_status()
            import time
            time.sleep(0.5)

        result = result_future.result()
        
        if result.result.error_code.val == 1: # SUCCESS
            return True
        else:
            self.get_logger().error(f"MoveIt Error Code: {result.result.error_code.val}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MoveToNamedTargetActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
