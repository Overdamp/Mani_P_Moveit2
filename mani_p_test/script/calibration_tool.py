#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import math
import sys
import time

class CalibrationTool(Node):
    def __init__(self):
        super().__init__('calibration_tool')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Frames to monitor
        self.camera_frame = "zed_left_camera_optical_frame" # Adjust if needed
        self.shelf_tag_frame = "tag36h11:0" # Shelf Tag (Example)
        self.cube_tag_frame = "tag36h11:2"  # Cube Tag (Example)
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("📏 Calibration Tool Started")
        self.get_logger().info(f"   Camera: {self.camera_frame}")
        self.get_logger().info("   Waiting for TF data...")

    def timer_callback(self):
        # 1. Camera <-> Shelf Tag
        self.print_transform(self.camera_frame, self.shelf_tag_frame, "Camera -> Shelf Tag")
        
        # 2. Camera <-> Cube Tag
        self.print_transform(self.camera_frame, self.cube_tag_frame, "Camera -> Cube Tag")
        
        # 3. Shelf Tag <-> Cube Tag (Relative position)
        self.print_transform(self.shelf_tag_frame, self.cube_tag_frame, "Shelf Tag -> Cube Tag")
        
        print("-" * 50)

    def print_transform(self, parent, child, label):
        try:
            t = self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            dist = math.sqrt(x**2 + y**2 + z**2)
            
            print(f"[{label}]")
            print(f"   Frames: {parent} -> {child}")
            print(f"   XYZ: ({x:.4f}, {y:.4f}, {z:.4f})")
            print(f"   Distance: {dist:.4f} m")
            
        except Exception as e:
            # Don't spam error if frame just missing
            pass

def main(args=None):
    rclpy.init(args=args)
    
    # Allow user to specify frames via args if needed
    # Usage: ros2 run mani_p_test calibration_tool.py [camera_frame] [shelf_tag] [cube_tag]
    
    node = CalibrationTool()
    
    if len(sys.argv) > 1:
        node.camera_frame = sys.argv[1]
    if len(sys.argv) > 2:
        node.shelf_tag_frame = sys.argv[2]
    if len(sys.argv) > 3:
        node.cube_tag_frame = sys.argv[3]
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
