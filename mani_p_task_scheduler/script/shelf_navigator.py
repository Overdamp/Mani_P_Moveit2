#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import sys
import math

class ShelfNavigator(Node):
    def __init__(self):
        super().__init__('shelf_navigator')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher for visualization
        self.goal_pub = self.create_publisher(PoseStamped, 'shelf_goal_pose', 10)
        
        # Shelf Configuration
        self.ref_tag = "tag36h11:11" # Reference Tag (Top Center)
        self.row_spacing = 0.40      # 40 cm
        self.col_spacing = 0.40      # 40 cm
        self.standoff_dist = 0.25    # 25 cm in front of shelf
        
        self.get_logger().info("📦 Shelf Navigator Started")
        self.get_logger().info(f"   Reference: {self.ref_tag}")
        self.get_logger().info(f"   Grid: 3 Rows x 3 Cols (Spacing: {self.row_spacing}m)")

    def get_target_pose(self, row, col):
        """
        Calculate target pose in Base_link frame.
        Row: 1 (Top), 2 (Middle), 3 (Bottom)
        Col: 1 (Left), 2 (Center), 3 (Right)
        """
        # 1. Calculate offsets in Tag Frame
        # Tag Frame: X=Right, Y=Down, Z=Out (Standard AprilTag)
        
        # Row Offsets (Y-axis)
        # Row 0 is Tag. Row 1 is 40cm down.
        y_offset = row * self.row_spacing
        
        # Col Offsets (X-axis)
        # Col 2 is Center (0). Col 1 is Left (-). Col 3 is Right (+).
        if col == 1:
            x_offset = -self.col_spacing
        elif col == 2:
            x_offset = 0.0
        elif col == 3:
            x_offset = self.col_spacing
        else:
            self.get_logger().error("Invalid Column! Use 1, 2, or 3.")
            return None

        # Z Offset (Standoff)
        z_offset = self.standoff_dist
        
        self.get_logger().info(f"🎯 Target relative to Tag: X={x_offset}, Y={y_offset}, Z={z_offset}")
        
        # 2. Create Pose in Tag Frame
        target_pose_tag = PoseStamped()
        target_pose_tag.header.frame_id = self.ref_tag
        target_pose_tag.header.stamp = self.get_clock().now().to_msg()
        target_pose_tag.pose.position.x = x_offset
        target_pose_tag.pose.position.y = y_offset
        target_pose_tag.pose.position.z = z_offset
        
        # Orientation: Gripper should face the shelf.
        # Tag Z points OUT. Gripper Z points OUT (usually).
        # So Gripper should oppose Tag Z? Or align?
        # Usually we want Gripper Z to point INTO the shelf for grasping.
        # But our ApproachTag logic usually aligns Z-Forward (Gripper) with Z-Forward (Tag) but flipped 180?
        # Let's use Identity orientation relative to Tag for now (Face same way as tag), 
        # but usually we need to rotate 180 around Y or X to face it.
        # For visualization, Identity is fine to see the point.
        target_pose_tag.pose.orientation.w = 1.0
        
        # 3. Transform to Base_link
        try:
            # Wait for transform
            if not self.tf_buffer.can_transform('Base_link', self.ref_tag, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                self.get_logger().warn(f"⚠️ Cannot find transform Base_link -> {self.ref_tag}")
                return None
                
            transform = self.tf_buffer.lookup_transform('Base_link', self.ref_tag, rclpy.time.Time())
            target_pose_base = tf2_geometry_msgs.do_transform_pose(target_pose_tag.pose, transform)
            
            # Create stamped pose for return/publishing
            result_pose = PoseStamped()
            result_pose.header.frame_id = 'Base_link'
            result_pose.header.stamp = self.get_clock().now().to_msg()
            result_pose.pose = target_pose_base
            
            return result_pose
            
        except Exception as e:
            self.get_logger().error(f"Transform Error: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    node = ShelfNavigator()
    
    if len(sys.argv) < 3:
        print("Usage: ros2 run mani_p_task_scheduler shelf_navigator.py [row] [col]")
        print("  Row: 1-3")
        print("  Col: 1-3")
        return
        
    try:
        row = int(sys.argv[1])
        col = int(sys.argv[2])
    except ValueError:
        print("Error: Row and Col must be integers.")
        return

    # Spin once to update TF buffer
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)
        
    target = node.get_target_pose(row, col)
    
    if target:
        print("\n✅ Calculated Target Pose (Base_link):")
        print(f"   Position: ({target.pose.position.x:.4f}, {target.pose.position.y:.4f}, {target.pose.position.z:.4f})")
        print(f"   Orientation: ({target.pose.orientation.x:.4f}, {target.pose.orientation.y:.4f}, {target.pose.orientation.z:.4f}, {target.pose.orientation.w:.4f})")
        
        # Publish for RViz
        node.goal_pub.publish(target)
        print("\n📡 Published to /shelf_goal_pose. Check RViz!")
        
        # Keep node alive briefly to ensure publish goes through
        import time
        time.sleep(1.0)
        
    else:
        print("\n❌ Failed to calculate target. Is the Tag visible?")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
