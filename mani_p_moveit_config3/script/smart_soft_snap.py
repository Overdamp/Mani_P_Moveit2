#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from tf2_ros import Buffer, TransformListener
import os
import struct
import math
from collections import deque
from ament_index_python.packages import get_package_share_directory

class ShelfMeshSpawner(Node):

    def __init__(self):
        super().__init__('shelf_mesh_spawner')

        # ==========================================
        # 🏎️ PERFORMANCE CONFIG 🏎️
        # ==========================================
        
        self.pkg_name = 'mani_p_description' 
        self.stl_filename = 'work_shelf.STL'
        self.tag_frame = "tag11"
        self.base_frame = "Base_link"
        
        # 1. ความไว (Latency Tuning)
        # ลดเหลือ 3-4 เพื่อความ Realtime สุดๆ (แต่ยังพอกรอง Noise ได้บ้าง)
        self.buffer_size = 4
        
        # 2. ความถี่ (Loop Rate)
        # 0.1 = 10Hz (ลดลงเพื่อไม่ให้ MoveIt รับภาระหนักเกินไป)
        self.timer_period = 0.1

        # --- การตั้งค่าอื่นๆ คงเดิม ---
        self.yaw_multiplier = 1.0 
        self.yaw_calibration_bias = 0.0 
        self.stl_offset_roll  = math.pi / 2.0
        self.stl_offset_pitch = 0.0
        self.stl_offset_yaw   = 0.0
        self.pos_offset_x = -0.03
        self.pos_offset_y = 0.0
        self.pos_offset_z = -0.020

        # Soft Snap Config
        self.snap_deadzone_deg = 5.0  
        self.snap_transition_deg = 15.0 
        # ==========================================

        self.pos_x_buffer = deque(maxlen=self.buffer_size)
        self.pos_y_buffer = deque(maxlen=self.buffer_size)
        self.pos_z_buffer = deque(maxlen=self.buffer_size)
        self.yaw_sin_buffer = deque(maxlen=self.buffer_size)
        self.yaw_cos_buffer = deque(maxlen=self.buffer_size)

        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 🚀 OPTIMIZATION: Load STL ONCE at startup 🚀
        # อ่านไฟล์มารอไว้เลย ไม่ต้องอ่านใหม่ทุกรอบลูป
        try:
            pkg_path = get_package_share_directory(self.pkg_name)
            self.stl_path = os.path.join(pkg_path, 'meshes', self.stl_filename)
            self.get_logger().info(f"Loading STL from: {self.stl_path}")
            self.cached_mesh_msg = self.parse_stl_binary(self.stl_path)
            
            if self.cached_mesh_msg is None:
                self.get_logger().error("Failed to load STL file!")
        except Exception as e:
            self.get_logger().error(f"Error finding package: {e}")
            self.cached_mesh_msg = None

        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        
        # Start Timer
        self.timer = self.create_timer(self.timer_period, self.update_shelf)
        self.get_logger().info(f'Shelf Spawner (Ultra Realtime Mode) Started.')

    # --- Math Helpers ---
    def get_euler_from_quaternion(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def apply_soft_snap(self, current_yaw):
        deg = math.degrees(current_yaw)
        abs_deg = abs(deg)
        if abs_deg <= self.snap_deadzone_deg: return 0.0
        if abs_deg >= self.snap_transition_deg: return current_yaw
        ratio = (abs_deg - self.snap_deadzone_deg) / (self.snap_transition_deg - self.snap_deadzone_deg)
        return current_yaw * ratio

    def parse_stl_binary(self, filename):
        mesh_msg = Mesh()
        if not os.path.exists(filename): return None
        with open(filename, 'rb') as f:
            f.read(80); count_bytes = f.read(4)
            if len(count_bytes) < 4: return None
            num_triangles = struct.unpack('<I', count_bytes)[0]
            vertex_index = 0
            for _ in range(num_triangles):
                data = f.read(50)
                if len(data) < 50: break
                floats = struct.unpack('<3f3f3f', data[12:48])
                p1 = Point(x=float(floats[0]), y=float(floats[1]), z=float(floats[2]))
                p2 = Point(x=float(floats[3]), y=float(floats[4]), z=float(floats[5]))
                p3 = Point(x=float(floats[6]), y=float(floats[7]), z=float(floats[8]))
                mesh_msg.vertices.extend([p1, p2, p3])
                tri = MeshTriangle(); tri.vertex_indices = [vertex_index, vertex_index+1, vertex_index+2]
                mesh_msg.triangles.append(tri); vertex_index += 3
        return mesh_msg

    def update_shelf(self):
        # ถ้าไม่มี Mesh ก็ไม่ต้องทำอะไร (Safety Check)
        if self.cached_mesh_msg is None: return

        try:
            # Timeout สั้นจู๋ (0.02) เพื่อ Loop ไม่สะดุด
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.tag_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.02))

            raw_x = transform.transform.translation.x
            raw_y = transform.transform.translation.y
            raw_z = transform.transform.translation.z
            _, _, raw_yaw = self.get_euler_from_quaternion(transform.transform.rotation)

            # Buffer
            self.pos_x_buffer.append(raw_x)
            self.pos_y_buffer.append(raw_y)
            self.pos_z_buffer.append(raw_z)
            self.yaw_sin_buffer.append(math.sin(raw_yaw))
            self.yaw_cos_buffer.append(math.cos(raw_yaw))

            if len(self.pos_x_buffer) < 1: return

            # Fast Average
            avg_x = sum(self.pos_x_buffer) / len(self.pos_x_buffer)
            avg_y = sum(self.pos_y_buffer) / len(self.pos_y_buffer)
            avg_z = sum(self.pos_z_buffer) / len(self.pos_z_buffer)
            avg_sin = sum(self.yaw_sin_buffer) / len(self.yaw_sin_buffer)
            avg_cos = sum(self.yaw_cos_buffer) / len(self.yaw_cos_buffer)
            avg_yaw_raw = math.atan2(avg_sin, avg_cos)

            # Logic คำนวณมุม (Soft Snap)
            adjusted_yaw = (avg_yaw_raw * self.yaw_multiplier) + self.yaw_calibration_bias
            final_yaw_base = self.apply_soft_snap(adjusted_yaw)

            # Build Object (ใช้ Cached Mesh)
            shelf = CollisionObject()
            shelf.header.frame_id = self.base_frame
            shelf.id = "shelf_mesh"

            # 🚀 ตรงนี้ไม่อ่านไฟล์ใหม่แล้ว ใช้ของเดิมที่โหลดไว้
            shelf.meshes.append(self.cached_mesh_msg)

            mesh_pose = PoseStamped()
            mesh_pose.pose.position.x = avg_x + self.pos_offset_x
            mesh_pose.pose.position.y = avg_y + self.pos_offset_y
            mesh_pose.pose.position.z = avg_z + self.pos_offset_z
            
            final_roll  = 0.0 + self.stl_offset_roll
            final_pitch = 0.0 + self.stl_offset_pitch
            final_yaw   = final_yaw_base + self.stl_offset_yaw
            
            q_list = self.get_quaternion_from_euler(final_roll, final_pitch, final_yaw)
            mesh_pose.pose.orientation = Quaternion(x=q_list[0], y=q_list[1], z=q_list[2], w=q_list[3])
            shelf.mesh_poses.append(mesh_pose.pose)
            
            shelf.operation = CollisionObject.ADD
            self.collision_pub.publish(shelf)

        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ShelfMeshSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()