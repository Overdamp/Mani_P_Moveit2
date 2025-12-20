#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from geometry_msgs.msg import Pose  # นำเข้า message types
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition สำหรับ Cartesian path
from moveit_msgs.action import ExecuteTrajectory  # นำเข้า action definition สำหรับการเคลื่อนที่ตาม trajectory
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys

class CartesianPusher(Node):

    def __init__(self):
        super().__init__('cartesian_pusher')  # สร้าง Node ชื่อ 'cartesian_pusher'
        
        # --- ⚙️ CONFIG ⚙️ ---
        self.arm_group_name = "arm"       # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"         # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"     # ชื่อ frame อ้างอิง
        # --------------------

        # Action Client สำหรับการสั่งเคลื่อนที่ตาม Trajectory
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        # Service Client สำหรับคำนวณ Cartesian Path
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # TF Buffer และ Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Cartesian Pusher Ready.')  # แสดงข้อความพร้อมทำงาน

    def move_linear(self, distance):
        """ สั่งให้มือจับเคลื่อนที่ไปข้างหน้า (แกน Z) ในแบบ Cartesian """
        self.get_logger().info(f"📏 Cartesian Move: {distance}m")

        # 1. ดึงค่า Pose ปัจจุบัน (พร้อมระบบ Retry)
        t = None
        for i in range(10):
            try:
                # พยายามดึงค่า Transform
                t = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.ee_link,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                break # เจอแล้ว!
            except Exception as e:
                self.get_logger().warn(f"⏳ Waiting for TF... ({i+1}/10)")
                rclpy.spin_once(self, timeout_sec=0.5) # หมุน loop รอ TF
        
        if t is None:
            self.get_logger().error(f"❌ Could not get current pose after retries.")  # แจ้ง error ถ้าหาไม่เจอ
            return

        # 2. คำนวณ Pose เป้าหมาย (ปัจจุบัน + Offset ตามแกน Z ของตัวเอง)
        # ดึงค่า Quaternion ปัจจุบัน
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        
        # คำนวณระยะขจัดตามแกน Z ของ Local Frame
        dx = 2 * (qx*qz + qy*qw) * distance
        dy = 2 * (qy*qz - qx*qw) * distance
        dz = (1 - 2 * (qx*qx + qy*qy)) * distance
        
        # สร้าง Pose เป้าหมาย
        target_pose = Pose()
        target_pose.position.x = t.transform.translation.x + dx
        target_pose.position.y = t.transform.translation.y + dy
        target_pose.position.z = t.transform.translation.z + dz
        target_pose.orientation = t.transform.rotation # รักษา Orientation เดิมไว้

        # 3. เรียกใช้ Service เพื่อคำนวณ Cartesian Path
        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = [target_pose]
        req.max_step = 0.01 # ความละเอียด 1cm
        req.jump_threshold = 0.0 # ปิดการตรวจสอบ jump
        req.avoid_collisions = True # เปิดการหลบหลีกการชน

        self.get_logger().info("⏳ Computing Cartesian Path...")
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Path Planning Failed: {response.error_code.val}")  # แจ้ง error ถ้าวางแผนไม่สำเร็จ
            return
            
        if response.fraction < 0.9:
            self.get_logger().warn(f"⚠️ Path truncated! Only computed {response.fraction*100:.1f}%")  # แจ้งเตือนถ้า path ไม่ครบ

        # 4. สั่งเคลื่อนที่ตาม Trajectory (Execute)
        self.get_logger().info(f"🚀 Executing Path ({len(response.solution.joint_trajectory.points)} points)...")
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        self._execute_client.wait_for_server()
        send_goal_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('❌ Execution Rejected!')  # ถ้าถูกปฏิเสธ
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('✅ SUCCESS: Cartesian Move Complete!')  # สำเร็จ
        else:
            self.get_logger().error(f'❌ Execution Failed: {result.error_code.val}')  # ล้มเหลว

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    if len(sys.argv) < 2:
        print("\n⚠️  Usage Error!")
        print("   Syntax: ros2 run <pkg> cartesian_push.py [DISTANCE]")
        print("   Example: ros2 run mani_p_moveit_config3 cartesian_push.py 0.15\n")
        return

    dist = 0.15  # ค่า default
    try:
        dist = float(sys.argv[1])  # รับค่าระยะทางจาก argument
    except ValueError:
        print("⚠️ Warning: Invalid distance format. Using default 0.15m")

    pusher = CartesianPusher()
    pusher.move_linear(dist)  # สั่งเคลื่อนที่
    
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
