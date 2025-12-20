#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF
import time  # นำเข้า time
import sys  # นำเข้า sys

class ShelfAlignmentDebugger(Node):
    def __init__(self):
        super().__init__('shelf_alignment_debugger')  # สร้าง Node ชื่อ 'shelf_alignment_debugger'
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)  # สร้าง Timer เรียก callback ทุก 1 วินาที
        
        self.get_logger().info("🔍 Shelf Alignment Debugger Started")
        self.get_logger().info("PLEASE MANUALLY JOG THE ROBOT TO THE DESIRED 'ROW 1, COL 2' (Top Center) POSITION.")
        self.get_logger().info("I will tell you the coordinates relative to the Tag.")

        # Joint State Subscription (รับค่า Joint State)
        from sensor_msgs.msg import JointState
        self.current_joints = {}
        self.joint_sub = self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

    def joint_callback(self, msg):
        # Callback สำหรับเก็บค่า Joint ปัจจุบัน
        for i, name in enumerate(msg.name):
            self.current_joints[name] = msg.position[i]

    def timer_callback(self):
        try:
            # 1. Check if Tag is visible (ตรวจสอบว่ามองเห็น Tag หรือไม่)
            if not self.tf_buffer.can_transform('Base_link', 'tag11', rclpy.time.Time()):
                self.get_logger().warn("❌ Tag 'tag11' not visible!")
                return

            # 2. Get TCP relative to Tag (หาตำแหน่ง TCP เทียบกับ Tag)
            transform = self.tf_buffer.lookup_transform('tag11', 'tcp_link', rclpy.time.Time())
            
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            
            rx = transform.transform.rotation.x
            ry = transform.transform.rotation.y
            rz = transform.transform.rotation.z
            rw = transform.transform.rotation.w
            
            # 3. Get Joints (แสดงค่า Joint)
            joints_str = ", ".join([f"{k}: {v:.3f}" for k, v in self.current_joints.items() if 'J' in k or 'palm' in k])
            
            print("\n" + "="*40)
            print(f"🤖 ROBOT TCP RELATIVE TO TAG 'tag11'")
            print(f"   X (Right/Left): {tx:.3f} m")
            print(f"   Y (Up/Down)   : {ty:.3f} m")
            print(f"   Z (In/Out)    : {tz:.3f} m")
            print("-" * 20)
            print(f"   Rotation      : [{rx:.3f}, {ry:.3f}, {rz:.3f}, {rw:.3f}]")
            print("-" * 20)
            print(f"   Joints        : {{{joints_str}}}")
            print("="*40)
            print("👉 Use these X, Y, Z values to update 'shelf_action_server.py'")
            
        except Exception as e:
            pass

def main():
    rclpy.init()  # เริ่มต้น ROS 2
    node = ShelfAlignmentDebugger()
    try:
        rclpy.spin(node)  # หมุน loop
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
