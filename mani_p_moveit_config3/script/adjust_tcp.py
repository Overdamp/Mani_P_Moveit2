#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy สำหรับการเขียน ROS 2 node
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion  # นำเข้า message types
from moveit_msgs.action import MoveGroup  # นำเข้า action definition สำหรับ MoveGroup
from moveit_msgs.srv import GetCartesianPath  # นำเข้า service definition สำหรับ Cartesian path
from moveit_msgs.msg import Constraints, OrientationConstraint  # นำเข้า message types สำหรับข้อจำกัด
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import sys  # นำเข้า sys
import math  # นำเข้า math
import argparse  # นำเข้า argparse สำหรับจัดการ argument
import tf_transformations  # นำเข้าไลบรารีคำนวณ transformation
import numpy  # นำเข้า numpy สำหรับการคำนวณทางคณิตศาสตร์

class TCPAdjuster(Node):
    def __init__(self):
        super().__init__('tcp_adjuster')  # สร้าง Node ชื่อ 'tcp_adjuster'
        
        self.arm_group_name = "arm"  # ชื่อกลุ่มแขนกล
        self.ee_link = "tcp_link"  # ชื่อ link ปลายมือจับ
        self.base_frame = "Base_link"  # ชื่อ frame อ้างอิง
        
        # สร้าง Action Client สำหรับ MoveGroup
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.tf_buffer = Buffer()  # สร้าง Buffer เก็บ TF
        self.tf_listener = TransformListener(self.tf_buffer, self)  # สร้าง Listener รับ TF
        
        self.get_logger().info('TCP Adjuster Ready.')  # แสดงข้อความพร้อมทำงาน

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        # ฟังก์ชันแปลง Euler เป็น Quaternion
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    def move_to_tag(self, tag_id, offset):
        # ฟังก์ชันหลักในการเคลื่อนที่ไปหา Tag
        tag_frame = f"tag{tag_id}_fisheye_level"  # สร้างชื่อ frame ของ tag ที่ต้องการหา
        
        self.get_logger().info(f"🔍 Looking for {tag_frame}...")  # แสดงข้อความกำลังหา Tag
        
        # รอรับค่า TF (Spinning เพื่อให้ buffer ได้รับข้อมูล)
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                # พยายามดึงค่า Transform ของ Tag เทียบกับ Base_link
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_frame, rclpy.time.Time())
                break # เจอแล้ว! ออกจาก loop
            except Exception:
                # ถ้าหาไม่เจอภายใน 5 วินาที ให้แจ้ง error และจบการทำงาน
                if (self.get_clock().now() - start_time).nanoseconds > 5e9: # 5 วินาที
                    self.get_logger().error(f"❌ Timeout waiting for {tag_frame}")
                    return
                pass

        # เก็บค่าตำแหน่งและทิศทางของ Tag
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        tr = t.transform.rotation
        
        # แปลง Quaternion ของ Tag เป็น Rotation Matrix
        tag_q = [tr.x, tr.y, tr.z, tr.w]
        tag_matrix = tf_transformations.quaternion_matrix(tag_q)
        
        # คำนวณตำแหน่งเป้าหมายโดยการ Offset ออกมาจากหน้า Tag (แกน Z ของ Tag)
        # เราต้องการอยู่ห่างจากหน้า Tag เป็นระยะ 'offset' เมตร
        target_x = tx + (tag_matrix[0, 2] * offset)
        target_y = ty + (tag_matrix[1, 2] * offset)
        target_z = tz + (tag_matrix[2, 2] * offset)
        
        # ผู้ใช้ต้องการให้สูงขึ้นจากชั้นวาง (Offset แกน Z แนวตั้ง)
        VERTICAL_OFFSET = 0.025
        target_z += VERTICAL_OFFSET
        
        # ตรรกะการจัดทิศทาง (Orientation Logic):
        # เราต้องการให้ TCP หันหน้าเข้าหา Tag (แกน Z ของ TCP สวนทางกับแกน Z ของ Tag)
        # แต่ต้องการรักษาระนาบ "บน" ของ TCP ให้สอดคล้องกับโลก (World)
        # เพื่อไม่ให้มือจับหมุนกลับหัวเมื่อ Tag เอียง
        
        # 1. คำนวณเวกเตอร์จาก TCP ไปยัง Tag (Direction Vector)
        # เนื่องจากเราต้องการเล็งไปที่แกน Z ของ Tag (ซึ่งชี้ออกมา)
        # ดังนั้นเวกเตอร์เข้าหาคือ -Tag_Z (ชี้เข้าไปใน Tag)
        
        # แกน Z ของ Tag ใน World Frame
        tag_z_axis = tag_matrix[:3, 2] # [x, y, z]
        
        # แกน Z เป้าหมายของ TCP = -Tag Z axis (หันหน้าชนกัน)
        target_tcp_z = -tag_z_axis
        
        # แกน Z ของโลก (ทิศขึ้น)
        world_z = [0, 0, 1]
        
        # แกน Y ของ TCP (หรือ X) ควรตั้งฉากกับ TCP Z และ World Z
        # เพื่อรักษามือจับให้ "ขนาน" กับพื้นโลก
        tcp_y_vec = numpy.cross(tcp_z_vec, world_z)
        norm_y = numpy.linalg.norm(tcp_y_vec)
        
        if norm_y < 0.001:
            # กรณี Singularity: TCP Z ขนานกับ World Z (มองขึ้นฟ้าหรือลงดินตรงๆ)
            # ให้ใช้แกน Y default ไปเลย
            tcp_y_vec = [0, 1, 0] # Default
        else:
            tcp_y_vec = tcp_y_vec / norm_y  # Normalize ให้เป็น Unit vector
            
        # TCP X = Cross Product ของ (Y, Z)
        tcp_x_vec = numpy.cross(tcp_y_vec, tcp_z_vec)
        tcp_x_vec = tcp_x_vec / numpy.linalg.norm(tcp_x_vec)
        
        # สร้าง Rotation Matrix จากแกน X, Y, Z ที่คำนวณได้
        R = numpy.identity(4)
        R[0, 0] = tcp_x_vec[0]; R[0, 1] = tcp_y_vec[0]; R[0, 2] = tcp_z_vec[0]
        R[1, 0] = tcp_x_vec[1]; R[1, 1] = tcp_y_vec[1]; R[1, 2] = tcp_z_vec[1]
        R[2, 0] = tcp_x_vec[2]; R[2, 1] = tcp_y_vec[2]; R[2, 2] = tcp_z_vec[2]
        
        # แปลง Matrix เป็น Quaternion
        target_q_np = tf_transformations.quaternion_from_matrix(R)
        
        # ผู้ใช้ต้องการ: ก้มลง (Pitch down) 10 องศา
        # หมายเหตุ: การทดลองก่อนหน้า (+5 และ -5) อาจจะยังไม่ชัดเจน
        # ลองใช้ +10.0 องศาเพื่อให้เห็นผลชัดเจน
        pitch_offset_rad = math.radians(10.0) 
        q_pitch = tf_transformations.quaternion_from_euler(0, pitch_offset_rad, 0)
        
        # รวมการหมุน: q_new = q_old * q_pitch
        target_q_np = tf_transformations.quaternion_multiply(target_q_np, q_pitch)
        
        # สร้าง PoseStamped เป้าหมาย
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z
        target_pose.pose.orientation.x = target_q_np[0]
        target_pose.pose.orientation.y = target_q_np[1]
        target_pose.pose.orientation.z = target_q_np[2]
        target_pose.pose.orientation.w = target_q_np[3]
        
        self.get_logger().info(f"🎯 Target: {tag_frame} + {offset}m (+{VERTICAL_OFFSET}m height)")
        self.get_logger().info(f"   Pos: ({target_x:.3f}, {target_y:.3f}, {target_z:.3f})")
        self.get_logger().info("   Orientation: Level Horizon + 10deg Pitch")
        
        # ---------------------------------------------------------
        # ตรรกะ Cartesian Path (การเคลื่อนที่แบบเส้นตรง)
        # ---------------------------------------------------------
        
        # 1. สร้าง Service Client
        self._cartesian_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        if not self._cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("❌ Service 'compute_cartesian_path' not available")
            return

        # 2. เตรียม Waypoints
        # เรามีแค่ waypoint เดียวคือจุดเป้าหมาย
        
        # ดึง Pose จาก PoseStamped
        target_pose_msg = target_pose.pose
        waypoints = [target_pose_msg]

        req = GetCartesianPath.Request()
        req.header.frame_id = self.base_frame
        req.header.stamp = self.get_clock().now().to_msg()
        req.group_name = self.arm_group_name
        req.link_name = self.ee_link
        req.waypoints = waypoints
        req.max_step = 0.01       # ความละเอียด 1cm
        req.jump_threshold = 0.0  # ปิดการตรวจสอบ jump
        req.avoid_collisions = True # เปิดการหลบหลีกการชน

        self.get_logger().info("📏 Computing Cartesian Path...")
        future = self._cartesian_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(f"❌ Cartesian Planning Failed: {response.error_code.val}")
            # ถ้าวางแผนล้มเหลว ให้จบการทำงาน
            return
            
        fraction = response.fraction  # สัดส่วนของ path ที่คำนวณได้ (0.0 - 1.0)
        self.get_logger().info(f"   Path Fraction: {fraction * 100:.1f}%")
        
        if fraction < 0.95:
             # ถ้าคำนวณ path ได้ไม่ครบ (น้อยกว่า 95%) ให้แจ้งเตือนและใช้แผนสำรอง (PTP)
             self.get_logger().warn(f"⚠️ Cartesian Path incomplete ({fraction*100:.1f}%)! Falling back to PTP...")
             
             # Fallback: ใช้ MoveGroup แบบปกติ (PTP - Point to Point)
             # อาจจะไม่เป็นเส้นตรง แต่จะไปถึงเป้าหมายได้
             
             # สร้าง Constraints สำหรับ PTP
             constraints = Constraints()
             from moveit_msgs.msg import PositionConstraint, OrientationConstraint, BoundingVolume
             from shape_msgs.msg import SolidPrimitive
             
             # 1. Position Constraint
             pc = PositionConstraint()
             pc.header.frame_id = self.base_frame
             pc.link_name = self.ee_link
             pc.target_point_offset.x = 0.0
             pc.target_point_offset.y = 0.0
             pc.target_point_offset.z = 0.0
             
             box = SolidPrimitive()
             box.type = SolidPrimitive.BOX
             box.dimensions = [0.001, 0.001, 0.001] 
             pc.constraint_region.primitives.append(box)
             pc.constraint_region.primitive_poses.append(target_pose.pose)
             pc.weight = 1.0
             
             # 2. Orientation Constraint
             oc = OrientationConstraint()
             oc.header.frame_id = self.base_frame
             oc.link_name = self.ee_link
             oc.orientation = target_pose.pose.orientation
             oc.absolute_x_axis_tolerance = 0.01
             oc.absolute_y_axis_tolerance = 0.01
             oc.absolute_z_axis_tolerance = 0.01
             oc.weight = 1.0
             
             constraints.position_constraints.append(pc)
             constraints.orientation_constraints.append(oc)
             
             # ส่ง Goal ไปยัง MoveGroup
             goal_msg = MoveGroup.Goal()
             goal_msg.request.group_name = self.arm_group_name
             goal_msg.request.num_planning_attempts = 10
             goal_msg.request.allowed_planning_time = 5.0
             goal_msg.request.max_velocity_scaling_factor = 0.1
             goal_msg.request.max_acceleration_scaling_factor = 0.1
             goal_msg.request.goal_constraints.append(constraints)
             
             self.get_logger().info("🚀 Sending PTP Goal (Fallback)...")
             self._action_client.wait_for_server()
             future = self._action_client.send_goal_async(goal_msg)
             rclpy.spin_until_future_complete(self, future)
             goal_handle = future.result()
             
             if not goal_handle.accepted:
                 self.get_logger().error('❌ PTP Goal Rejected!')
                 return

             res_future = goal_handle.get_result_async()
             rclpy.spin_until_future_complete(self, res_future)
             result = res_future.result().result
             
             if result.error_code.val == 1:
                 self.get_logger().info('✅ Arrived (PTP Fallback)!')
             else:
                 self.get_logger().error(f'❌ PTP Failed: {result.error_code.val}')
             return

        # 3. Execute Cartesian
        # ถ้า Cartesian path สมบูรณ์ ให้สั่งเคลื่อนที่ตาม trajectory ที่ได้
        from moveit_msgs.action import ExecuteTrajectory
        self._execute_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self._execute_client.wait_for_server()
        
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = response.solution
        
        self.get_logger().info("🚀 Executing Linear Move...")
        send_future = self._execute_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal Rejected!')
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        
        if result.error_code.val == 1:
            self.get_logger().info('✅ Arrived (Linear)!')
        else:
            self.get_logger().error(f'❌ Failed: {result.error_code.val}')

def main():
    rclpy.init()  # เริ่มต้น ROS 2
    
    # ตั้งค่า Argument Parser
    parser = argparse.ArgumentParser(description="Move TCP to tag_fisheye_level")
    parser.add_argument("tag_id", type=int, help="Tag ID (1, 2, 3)")
    parser.add_argument("--offset", type=float, default=0.1, help="Distance from tag (meters). Default 0.1")
    
    # กรอง ROS args ออกเพื่อให้ argparse ทำงานได้ถูกต้อง
    args = parser.parse_args([arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')])
    
    node = TCPAdjuster()  # สร้าง Node
    node.move_to_tag(args.tag_id, args.offset)  # เรียกฟังก์ชันเคลื่อนที่
    node.destroy_node()  # ทำลาย Node
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
