#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import sys
import csv
import time
import math
import argparse
from datetime import datetime

class TFRecorder(Node):

    def __init__(self, target_frames, duration, output_file):
        super().__init__('tf_recorder')
        
        self.target_frames = target_frames
        self.duration = duration
        self.output_file = output_file
        self.base_frame = "Base_link"
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.start_time = time.time()
        self.records = []
        
        self.get_logger().info(f"🎙️ Recording TF for {duration}s...")
        self.get_logger().info(f"   Targets: {target_frames}")
        self.get_logger().info(f"   Output: {output_file}")

        self.timer = self.create_timer(0.1, self.record_callback) # 10Hz

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

    def record_callback(self):
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        if elapsed > self.duration:
            self.save_and_exit()
            return

        timestamp = datetime.now().strftime("%H:%M:%S.%f")
        
        for frame in self.target_frames:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.base_frame, frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))
                
                tx = t.transform.translation.x
                ty = t.transform.translation.y
                tz = t.transform.translation.z
                rx, ry, rz = self.get_euler_from_quaternion(t.transform.rotation)
                
                # Format: [timestamp, elapsed, frame_id, x, y, z, roll, pitch, yaw]
                row = [timestamp, f"{elapsed:.2f}", frame, f"{tx:.4f}", f"{ty:.4f}", f"{tz:.4f}", f"{rx:.4f}", f"{ry:.4f}", f"{rz:.4f}"]
                self.records.append(row)
                
            except Exception as e:
                # self.get_logger().warn(f"Missing transform for {frame}")
                pass

    def save_and_exit(self):
        self.get_logger().info("💾 Saving data...")
        try:
            with open(self.output_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "elapsed_time", "frame_id", "x", "y", "z", "roll", "pitch", "yaw"])
                writer.writerows(self.records)
            self.get_logger().info(f"✅ Data saved to {self.output_file}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to save: {e}")
        
        rclpy.shutdown()

def main():
    rclpy.init()
    
    parser = argparse.ArgumentParser(description="Record TF data to CSV")
    parser.add_argument("frames", nargs='+', help="List of frames to record (e.g. tag11 shelf_filtered)")
    parser.add_argument("--duration", type=float, default=10.0, help="Recording duration in seconds")
    parser.add_argument("--output", type=str, default="tf_data.csv", help="Output filename")
    
    # ROS2 passes unparsed args, so we need to filter them if running via ros2 run
    # But argparse handles sys.argv automatically. Let's just use sys.argv carefully.
    # A cleaner way for ROS2 nodes is to use parameters, but argparse is fine for a script.
    
    # Hack to remove ros args if present
    clean_args = [arg for arg in sys.argv[1:] if not arg.startswith('--ros-args')]
    args = parser.parse_args(clean_args)

    recorder = TFRecorder(args.frames, args.duration, args.output)
    rclpy.spin(recorder)

if __name__ == '__main__':
    main()
