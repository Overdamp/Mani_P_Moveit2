#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy สำหรับการเขียน ROS 2 node
from rclpy.node import Node  # นำเข้าคลาส Node เพื่อสร้าง ROS 2 node
from rclpy.action import ActionClient  # นำเข้า ActionClient เพื่อเรียกใช้ action server
from geometry_msgs.msg import Pose, PoseStamped  # นำเข้า message types สำหรับตำแหน่งและท่าทาง
from moveit_msgs.action import MoveGroup  # นำเข้า action definition สำหรับ MoveGroup
from moveit_msgs.msg import Constraints, OrientationConstraint  # นำเข้า message types สำหรับข้อจำกัดการเคลื่อนที่
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition สำหรับการคำนวณ Cartesian path
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีสำหรับจัดการ TF (Transform)
import sys  # นำเข้า sys สำหรับจัดการ argument จาก command line
import math  # นำเข้า math สำหรับการคำนวณทางคณิตศาสตร์
import time  # นำเข้า time สำหรับการหน่วงเวลา

class LevelAdjuster(Node):
    def __init__(self):
        super().__init__('level_adjuster')  # เรียก constructor ของคลาสแม่ (Node) และตั้งชื่อ node ว่า 'level_adjuster'
        
        self.arm_group_name = "arm"  # กำหนดชื่อกลุ่มของแขนกล (Planning Group)
        self.ee_link = "tcp_link"  # กำหนดชื่อ link ปลายมือจับ (End Effector)
        self.base_frame = "Base_link"  # กำหนดชื่อ frame อ้างอิงหลัก (Base)
        
        # สร้าง Action Client สำหรับ MoveGroup เพื่อสั่งการเคลื่อนที่
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        # สร้าง Service Client สำหรับคำนวณ Cartesian path
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        
        self.tf_buffer = Buffer()  # สร้าง Buffer สำหรับเก็บข้อมูล Transform
        self.tf_listener = TransformListener(self.tf_buffer, self)  # สร้าง Listener เพื่อรับข้อมูล Transform ลงใน Buffer
        
        self.get_logger().info('Level Adjuster Ready.')  # แสดงข้อความว่า Node พร้อมทำงานแล้ว

    def euler_from_quaternion(self, q):
        # ฟังก์ชันแปลง Quaternion เป็น Euler angles (Roll, Pitch, Yaw)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)  # คำนวณค่า Roll

        sinp = 2 * (q.w * q.y - q.z * q.x)
        # คำนวณค่า Pitch โดยมีการตรวจสอบขอบเขตเพื่อป้องกัน error
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)  # คำนวณค่า Yaw
        return roll, pitch, yaw  # คืนค่า Roll, Pitch, Yaw

    def quaternion_from_euler(self, roll, pitch, yaw):
        # ฟังก์ชันแปลง Euler angles เป็น Quaternion
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]  # คืนค่า Quaternion [x, y, z, w]

    def adjust_orientation(self, target_roll_deg=0.0):
        # ฟังก์ชันปรับทิศทางของปลายมือจับ
        # 1. ดึงค่า Pose ปัจจุบัน
        try:
            # รอและดึงค่า Transform จาก Base_link ไปยัง tcp_link
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not get current pose: {e}")  # แสดงข้อความ error หากดึงค่าไม่ได้
            return

        current_x = t.transform.translation.x  # เก็บค่าตำแหน่ง X ปัจจุบัน
        current_y = t.transform.translation.y  # เก็บค่าตำแหน่ง Y ปัจจุบัน
        current_z = t.transform.translation.z  # เก็บค่าตำแหน่ง Z ปัจจุบัน
        
        # 2. ดึงค่า Yaw ปัจจุบัน
        _, _, current_yaw = self.euler_from_quaternion(t.transform.rotation)  # แปลง Quaternion เป็น Euler เพื่อเอาค่า Yaw
        
        # 3. สร้าง Orientation ใหม่ (ให้ตรงกับ ZED Camera)
        # ความต้องการผู้ใช้: orientation ของ tcp_link ต้องตรงกับ zed_left_camera_optical_frame
        
        target_frame = "zed_mani_left_camera_optical_frame"  # กำหนด frame เป้าหมายที่ต้องการอ้างอิง
        try:
            # รอและดึงค่า Transform ของ frame เป้าหมาย
            t_zed = self.tf_buffer.lookup_transform(
                self.base_frame, target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
        except Exception as e:
            self.get_logger().error(f"❌ Could not lookup {target_frame}: {e}")  # แสดงข้อความ error หากหา frame ไม่เจอ
            return

        # เก็บค่า Quaternion ของ frame เป้าหมาย
        q_new = [
            t_zed.transform.rotation.x,
            t_zed.transform.rotation.y,
            t_zed.transform.rotation.z,
            t_zed.transform.rotation.w
        ]
        
        self.get_logger().info(f"🔄 Adjusting to match {target_frame}")  # แสดงข้อความกำลังปรับทิศทาง

        # 4. สร้าง Cartesian Path (Task Space)
        # ผู้ใช้ต้องการโหมด "task_space" เพื่อบังคับการจัดเรียงตัว
        
        target_pose = Pose()  # สร้าง object Pose เป้าหมาย
        target_pose.position.x = current_x  # ใช้ตำแหน่ง X เดิม
        target_pose.position.y = current_y  # ใช้ตำแหน่ง Y เดิม
        target_pose.position.z = current_z  # ใช้ตำแหน่ง Z เดิม
        target_pose.orientation.x = q_new[0]  # กำหนด Orientation X ใหม่
        target_pose.orientation.y = q_new[1]  # กำหนด Orientation Y ใหม่
        target_pose.orientation.z = q_new[2]  # กำหนด Orientation Z ใหม่
        target_pose.orientation.w = q_new[3]  # กำหนด Orientation W ใหม่
        
        waypoints = [target_pose]  # สร้างรายการ waypoints สำหรับ Cartesian path
        
        req = GetCartesianPath.Request()  # สร้าง Request สำหรับ service
        req.header.frame_id = self.base_frame  # กำหนด frame อ้างอิง
        req.header.stamp = self.get_clock().now().to_msg()  # ใส่ timestamp ปัจจุบัน
        req.group_name = self.arm_group_name  # กำหนดชื่อกลุ่มแขนกล
        req.link_name = self.ee_link  # กำหนดชื่อ link ปลายมือจับ
        req.waypoints = waypoints  # ใส่ waypoints ที่ต้องการ
        req.max_step = 0.01 # ความละเอียด 1cm
        req.jump_threshold = 0.0 # ปิดการตรวจสอบ jump สำหรับการหมุน
        req.avoid_collisions = True  # เปิดการหลบหลีกการชน
        
        self.get_logger().info("🔄 Computing Cartesian Path...")  # แสดงข้อความกำลังคำนวณ path
        future = self._cartesian_client.call_async(req)  # เรียกใช้ service แบบ async
        rclpy.spin_until_future_complete(self, future)  # รอจนกว่าจะเสร็จสิ้น
        response = future.result()  # รับผลลัพธ์
        
        if response.error_code.val != 1:  # ตรวจสอบว่าคำนวณสำเร็จหรือไม่ (1 = SUCCESS)
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")  # แสดง error ถ้าล้มเหลว
            return

        # สั่งการเคลื่อนที่ (Execute)
        from moveit_msgs.action import ExecuteTrajectory  # นำเข้า ExecuteTrajectory action
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')  # สร้าง Action Client
        
        goal_msg = ExecuteTrajectory.Goal()  # สร้าง Goal message
        goal_msg.trajectory = response.solution  # ใส่ trajectory ที่คำนวณได้ลงใน goal
        
        self._execute_client.wait_for_server()  # รอให้ server พร้อม
        send_future = self._execute_client.send_goal_async(goal_msg)  # ส่ง goal ไปยัง server
        rclpy.spin_until_future_complete(self, send_future)  # รอจนกว่าจะส่งเสร็จ
        goal_handle = send_future.result()  # รับ handle ของ goal
        
        if not goal_handle.accepted:  # ตรวจสอบว่า goal ถูกรับหรือไม่
            self.get_logger().error('❌ Goal Rejected!')  # แสดง error ถ้าถูกปฏิเสธ
            return

        self.get_logger().info('⏳ Executing Cartesian Move...')  # แสดงข้อความกำลังเคลื่อนที่
        res_future = goal_handle.get_result_async()  # รอผลลัพธ์การทำงาน
        rclpy.spin_until_future_complete(self, res_future)  # รอจนกว่าจะเสร็จ
        result = res_future.result().result  # รับผลลัพธ์สุดท้าย
        
        if result.error_code.val == 1:  # ตรวจสอบผลลัพธ์ (1 = SUCCESS)
            self.get_logger().info('✅ Adjustment Complete!')  # แสดงข้อความสำเร็จ
        else:
            self.get_logger().error(f'❌ Failed: {result.error_code.val}')  # แสดงข้อความล้มเหลว

        # ส่วนนี้ดูเหมือนจะซ้ำซ้อนกับด้านบน อาจจะเป็น code เก่าที่ลืมลบ แต่ผมจะ comment ไว้ตามเดิม
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
    rclpy.init(args=args)  # เริ่มต้นระบบ ROS 2
    
    roll_deg = 0.0  # ค่าเริ่มต้นของ Roll
    if len(sys.argv) > 1:  # ตรวจสอบว่ามี argument ส่งมาหรือไม่
        roll_deg = float(sys.argv[1])  # แปลง argument เป็น float
        
    node = LevelAdjuster()  # สร้าง instance ของ LevelAdjuster
    # หมุน loop สั้นๆ เพื่อให้ TF buffer ได้รับข้อมูล
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        
    node.adjust_orientation(roll_deg)  # เรียกฟังก์ชันปรับทิศทาง
    node.destroy_node()  # ทำลาย node เมื่อเสร็จสิ้น
    rclpy.shutdown()  # ปิดระบบ ROS 2

if __name__ == '__main__':
    main()  # เรียกฟังก์ชัน main เมื่อรันไฟล์
