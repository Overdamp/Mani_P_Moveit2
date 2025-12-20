#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionServer, ActionClient  # นำเข้า ActionServer และ ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup  # นำเข้า Callback Group แบบ Reentrant
from moveit_msgs.action import MoveGroup  # นำเข้า action definition สำหรับ MoveGroup
from moveit_msgs.msg import MoveItErrorCodes  # นำเข้า Error Codes ของ MoveIt
import time  # นำเข้า time

# Custom Action Interface (นำเข้า Action Interface ที่สร้างเอง)
from mani_p_actions.action import GripperControl
from sensor_msgs.msg import JointState  # นำเข้า message types สำหรับ Joint State
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF

class GripperActionServer(Node):

    def __init__(self):
        super().__init__('gripper_action_server')  # สร้าง Node ชื่อ 'gripper_action_server'
        
        self.cb_group = ReentrantCallbackGroup()  # สร้าง Callback Group เพื่อให้ทำงานขนานกันได้

        # สร้าง Action Server สำหรับ GripperControl
        self._action_server = ActionServer(
            self,
            GripperControl,
            'gripper_control',
            self.execute_callback,
            callback_group=self.cb_group
        )

        # MoveGroup Client for Gripper (สร้าง Client สำหรับ MoveGroup เพื่อคุม Gripper)
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.cb_group)
        
        self.gripper_group_name = "gripper"  # ชื่อกลุ่ม Gripper
        self.get_logger().info('✅ Gripper Action Server Ready (MoveIt Named Targets).')
        
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
        is_open = goal_handle.request.open
        target_name = "gripper_open" if is_open else "gripper_close"
        self.get_logger().info(f"🖐️ Gripper Command: {target_name}")
        
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup Action Server not available")
            goal_handle.abort()
            return GripperControl.Result(success=False)

        # Create MoveGroup Goal (สร้าง Goal สำหรับ MoveGroup)
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.gripper_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Set Named Target (กำหนดเป้าหมายตามชื่อ)
        # Note: MoveGroup.Goal doesn't have a direct 'named_target' field in the request root.
        # (หมายเหตุ: MoveGroup.Goal ไม่มีฟิลด์ named_target โดยตรง)
        
        # Values from config/Manipulator_station_urdf_2.srdf (ค่าจากไฟล์ SRDF)
        # gripper_open: finger_middle_joint_l = 0.45099
        # gripper_close: finger_middle_joint_l = -1.1 (Modified for even tighter grip)
        
        target_joint_val = 0.45099 if is_open else -1.1
        
        from moveit_msgs.msg import Constraints, JointConstraint
        
        constraints = Constraints()
        constraints.name = target_name
        
        jc = JointConstraint()
        jc.joint_name = "finger_middle_joint_l" 
        jc.position = target_joint_val
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        
        constraints.joint_constraints.append(jc)
        goal_msg.request.goal_constraints.append(constraints)
        
        send_future = self._move_group_client.send_goal_async(goal_msg)
        while not send_future.done():
            time.sleep(0.01)
        goal_handle_moveit = send_future.result()
        
        if not goal_handle_moveit.accepted:
            self.get_logger().error('Goal Rejected by MoveIt')
            goal_handle.abort()
            return GripperControl.Result(success=False)

        res_future = goal_handle_moveit.get_result_async()
        while not res_future.done():
            self.log_status()
            time.sleep(0.5)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info("✅ Gripper Success!")
            goal_handle.succeed()
            return GripperControl.Result(success=True)
        else:
            self.get_logger().error(f"❌ MoveIt Error Code: {result.error_code.val}")
            # Error Code 1 = SUCCESS
            # Error Code -1 = PLANNING_FAILED
            # Error Code -4 = SOLUTION_IS_INVALID
            # Error Code -21 = NO_IK_SOLUTION
            goal_handle.abort()
            return GripperControl.Result(success=False)

def main(args=None):
    rclpy.init(args=args)
    node = GripperActionServer()
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
