#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
import py_trees  # นำเข้า py_trees
import py_trees_ros  # นำเข้า py_trees_ros
import py_trees.console as console  # นำเข้า console สำหรับแสดงผลสี
import sys  # นำเข้า sys

from mani_p_actions.action import MoveToShelf, ApproachTag  # นำเข้า Action ที่สร้างเอง
from geometry_msgs.msg import PoseStamped  # นำเข้า message types

def create_root():
    # 1. Root Sequence (สร้าง Root Sequence)
    root = py_trees.composites.Sequence(name="Main Task", memory=True)

    # 2. Move To Shelf Action (Action ไปยังชั้นวาง)
    # py_trees_ros.actions.ActionClient is the standard way to call ROS 2 Actions
    move_shelf = py_trees_ros.actions.ActionClient(
        name="Move to Shelf",
        action_type=MoveToShelf,
        action_name="move_to_shelf",
        action_goal=MoveToShelf.Goal(row=1, col=2),
        generate_feedback_message=lambda msg: f"Moving: {msg.feedback.status}"
    )

    # 3. Approach Tag Action (Action เข้าหา Tag)
    approach = py_trees_ros.actions.ActionClient(
        name="Approach Tag",
        action_type=ApproachTag,
        action_name="approach_tag",
        action_goal=ApproachTag.Goal(tag_name="tag36h11:11", distance=0.25, roll_offset=0.0),
        generate_feedback_message=lambda msg: f"Approaching: {msg.feedback.current_state}"
    )

    # Add children to root (เพิ่มลูกเข้าสู่ Root)
    root.add_children([move_shelf, approach])
    return root

def main():
    rclpy.init(args=None)  # เริ่มต้น ROS 2
    
    # Create the tree (สร้าง Tree)
    root = create_root()
    
    # Create the ROS 2 Tree Manager (สร้างตัวจัดการ Tree สำหรับ ROS 2)
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    
    try:
        # Setup (Connect to ROS) (ตั้งค่าการเชื่อมต่อ ROS)
        tree.setup(timeout=15.0)
        
        # Tick the tree (Run the logic) (เริ่มทำงาน Tree)
        print("🌳 Ticking Tree...")
        tree.tick_tock(period_ms=500.0) # Tick every 500ms
        
        rclpy.spin(tree.node)
        
    except KeyboardInterrupt:
        print("Stopping tree...")
        tree.interrupt()
    except Exception as e:
        console.logerror(console.red + f"Error: {e}" + console.reset)
        
    rclpy.shutdown()  # ปิด ROS 2

if __name__ == '__main__':
    main()
