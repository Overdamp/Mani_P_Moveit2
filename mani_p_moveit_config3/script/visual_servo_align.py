#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from geometry_msgs.msg import Pose  # นำเข้า message types
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition
from moveit_msgs.action import ExecuteTrajectory  # นำเข้า action definition
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys
import math  # นำเข้า math
import time  # นำเข้า time

class VisualServoAligner(Node):

    def __init__(self):
        super().__init__('visual_servo_aligner')  # สร้าง Node ชื่อ 'visual_servo_aligner'
        
        # --- ⚙️ CONFIG ⚙️ (การตั้งค่า) ---
        self.arm_group_name = "arm"      # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"        # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"    # ชื่อ frame อ้างอิง
        
        # Tolerance (meters) (ค่าความคลาดเคลื่อนที่ยอมรับได้)
        self.tolerance = 0.002 # 2mm
        self.max_step = 0.02   # ระยะขยับสูงสุดต่อครั้ง (2cm)
        self.max_retries = 5   # จำนวนครั้งสูงสุดที่พยายามปรับ
        # --------------------

        # Action Client สำหรับสั่ง Execute Trajectory
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        
        # Service Client สำหรับคำนวณ Cartesian Path
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Visual Servo Aligner Ready.')

    def align_to_tag(self, tag_frame):
        """ Iteratively align tcp_link to tag_frame in X/Y axes """
        # ฟังก์ชันปรับตำแหน่ง TCP ให้ตรงกับ Tag ในแนวแกน X/Y แบบวนลูป
        self.get_logger().info(f"🎯 Aligning to {tag_frame}...")

        for attempt in range(self.max_retries):
            # 1. หาค่า Error (ตำแหน่ง Tag ใน Frame ของ TCP)
            try:
                # Look up Tag in TCP frame
                # ถ้า Tag อยู่ที่ (0.01, -0.01, 0.2) ใน TCP frame แปลว่า TCP ต้องขยับ (+0.01, -0.01) เพื่อให้ตรง
                t = self.tf_buffer.lookup_transform(
                    self.ee_link,
                    tag_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
            except Exception as e:
                self.get_logger().warn(f"⚠️ Could not see {tag_frame}. Is it in view?")
                return

            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dist_error = math.sqrt(dx*dx + dy*dy)

            self.get_logger().info(f"   Attempt {attempt+1}: Error X={dx:.4f}, Y={dy:.4f} (Dist={dist_error:.4f})")

            # 2. ตรวจสอบว่า Error อยู่ในเกณฑ์ที่ยอมรับได้หรือไม่
            if dist_error < self.tolerance:
                self.get_logger().info("✅ Aligned! Error is within tolerance.")
                return

            # 3. จำกัดระยะการขยับ (เพื่อความปลอดภัย)
            step_x = dx
            step_y = dy
            
            # Scale down if too large (ถ้าระยะห่างมากเกินไป ให้ขยับแค่ max_step)
            if dist_error > self.max_step:
                scale = self.max_step / dist_error
                step_x *= scale
                step_y *= scale
                self.get_logger().info(f"   ⚠️ Error too large, scaling step to {self.max_step}m")

            # 4. สั่งเคลื่อนที่สัมพัทธ์กับ TCP (Cartesian Move)
            self.move_relative(step_x, step_y, 0.0)
            
            # รอให้หุ่นยนต์นิ่งสักพัก
            time.sleep(1.0)

        self.get_logger().warn("❌ Max retries reached. Alignment might not be perfect.")

    def move_relative(self, x, y, z):
        """ Move tcp_link relative to itself """
        # ฟังก์ชันสั่งเคลื่อนที่ TCP เทียบกับตัวมันเอง
        
        # 1. ดึง Pose ปัจจุบันเทียบกับ Base Frame
        try:
            t_base = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")
            return

        # 2. คำนวณ Pose เป้าหมาย
        # เราต้องการขยับ (x,y,z) ใน TCP frame
        # ต้องหมุน vector นี้ให้เป็น Base frame ก่อน
        
        # Quaternion (Base -> TCP)
        qx = t_base.transform.rotation.x
        qy = t_base.transform.rotation.y
        qz = t_base.transform.rotation.z
        qw = t_base.transform.rotation.w
        
        # Rotate vector (x,y,z) by quaternion q
        # สูตรการหมุน Vector ด้วย Quaternion
        
        vx, vy, vz = x, y, z
        
        rx = (1 - 2*qy*qy - 2*qz*qz)*vx + (2*qx*qy - 2*qz*qw)*vy + (2*qx*qz + 2*qy*qw)*vz
        ry = (2*qx*qy + 2*qz*qw)*vx + (1 - 2*qx*qx - 2*qz*qz)*vy + (2*qy*qz - 2*qx*qw)*vz
        rz = (2*qx*qz - 2*qy*qw)*vx + (2*qy*qz + 2*qx*qw)*vy + (1 - 2*qx*qx - 2*qy*qy)*vz
        
        target_pose = Pose()
        target_pose.position.x = t_base.transform.translation.x + rx
        target_pose.position.y = t_base.transform.translation.y + ry
        target_pose.position.z = t_base.transform.translation.z + rz
        target_pose.orientation = t_base.transform.rotation # รักษา Orientation เดิมไว้

        # 3. ขอ Cartesian Path จาก MoveIt
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
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Path Planning Failed: {response.error_code.val}")
            return

        # 4. สั่ง Execute Trajectory
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

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    if len(sys.argv) < 2:
        print("\n⚠️  Usage Error!")
        print("   Syntax: ros2 run <pkg> visual_servo_align.py <TAG_NAME>")
        print("   Example: ros2 run mani_p_moveit_config3 visual_servo_align.py tag2\n")
        return

    tag_name = sys.argv[1]  # รับชื่อ Tag

    aligner = VisualServoAligner()
    aligner.align_to_tag(tag_name)  # เริ่มปรับตำแหน่ง
    
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
