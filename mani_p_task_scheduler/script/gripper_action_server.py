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

# Import SceneHelper & SRDFHelper (นำเข้า Helper)
from mani_p_task_scheduler.script.scene_helper import SceneHelper
from mani_p_task_scheduler.script.srdf_helper import SRDFHelper

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
        self.get_logger().info('✅ Gripper Action Server Ready (SRDF Named Targets).')
        
        # TF Setup (ตั้งค่า TF)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Joint State Subscription (รับค่า Joint State)
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)
        
        # Helpers
        self.scene_helper = SceneHelper(self)
        self.srdf_helper = SRDFHelper()

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
        target_id = goal_handle.request.target_id # รับชื่อวัตถุจาก Goal
        
        # Default target_id if empty (เพื่อความเข้ากันได้กับโค้ดเก่า)
        if not target_id:
            target_id = "cube1"
            
        target_name = "gripper_open" if is_open else "gripper_close"
        self.get_logger().info(f"🖐️ Gripper Command: {target_name} (Target: {target_id})")
        
        # --- Handle Attached Collision Object (จัดการ Attached Object) ---
        if not is_open:
            # Closing Gripper -> Attach Object (ถ้าสั่งปิดมือ -> Attach วัตถุ)
            finger_links = [
                "finger_middle_l", "finger_middle_r", 
                "finger_bottom_l", "finger_bottom_r", 
                "finger_link_bl", "finger_link_br",
                "palm_link"
            ]
            self.scene_helper.attach_box(object_id=target_id, link_name="palm_link", touch_links=finger_links)
        else:
            # Opening Gripper -> Detach Object (ถ้าสั่งเปิดมือ -> Detach วัตถุ)
            self.scene_helper.detach_box(object_id=target_id, frame_id="Base_link")
        # ----------------------------------------------------------------
        
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup Action Server not available")
            goal_handle.abort()
            return GripperControl.Result(success=False)

        # Create MoveGroup Goal (สร้าง Goal สำหรับ MoveGroup)
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.gripper_group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        
        # Get Joint Values from SRDF (ดึงค่า Joint จาก SRDF)
        vals = self.srdf_helper.get_pose('gripper', target_name)
        
        if vals is None:
             self.get_logger().warn(f"⚠️ Unknown named target in SRDF: '{target_name}'. Using Hardcoded Fallback.")
             # Fallback
             if is_open:
                 vals = {'finger_middle_joint_l': 0.45099}
             else:
                 vals = {'finger_middle_joint_l': -1.1}

        from moveit_msgs.msg import Constraints, JointConstraint
        
        constraints = Constraints()
        constraints.name = target_name
        
        for joint, angle in vals.items():
            jc = JointConstraint()
            jc.joint_name = joint
            jc.position = angle
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
