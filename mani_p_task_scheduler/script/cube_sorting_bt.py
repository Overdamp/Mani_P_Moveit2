#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
import py_trees  # นำเข้า py_trees (Behavior Tree Library)
import py_trees_ros  # นำเข้า py_trees_ros (ROS Wrapper สำหรับ py_trees)
import py_trees.console as console  # นำเข้า console สำหรับแสดงผลสี
import sys  # นำเข้า sys
import time  # นำเข้า time

# นำเข้า Custom Actions ที่สร้างไว้
from mani_p_actions.action import MoveToShelf, ApproachTag, VisualServo, MoveLinear, GripperControl
from geometry_msgs.msg import PoseStamped  # นำเข้า message types
from tf2_ros import Buffer, TransformListener  # นำเข้าไลบรารีจัดการ TF

# --- Custom Behaviors (สร้าง Behavior เอง) ---

class ScanForCube(py_trees.behaviour.Behaviour):
    def __init__(self, name="Scan For Cube"):
        super(ScanForCube, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()  # ใช้ Blackboard เพื่อแชร์ข้อมูลระหว่าง Node
        self.tf_buffer = None
        self.tf_listener = None

    def setup(self, **kwargs):
        # ตั้งค่าเริ่มต้น (ทำงานครั้งเดียวเมื่อเริ่ม Tree)
        self.node = kwargs.get('node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def update(self):
        # ฟังก์ชันทำงานหลัก (วนลูปทุก Tick)
        # Check for tags ID 1-6 (ตรวจสอบ Tag ID 1-6)
        for i in range(1, 7):
            tag_frame = f"tag36h11:{i}"
            try:
                # Check if transform exists (ตรวจสอบว่าเจอ TF ของ Tag หรือไม่)
                if self.tf_buffer.can_transform('Base_link', tag_frame, rclpy.time.Time()):
                    self.blackboard.set("detected_id", i)  # บันทึก ID ลง Blackboard
                    self.blackboard.set("detected_tag_name", tag_frame)  # บันทึกชื่อ Tag ลง Blackboard
                    self.node.get_logger().info(f"📦 Found Cube ID: {i}")
                    return py_trees.common.Status.SUCCESS  # เจอแล้ว ส่งคืน Success
            except Exception:
                pass
        
        self.node.get_logger().info("Scanning... No cube found.")
        return py_trees.common.Status.FAILURE  # ไม่เจอ ส่งคืน Failure

class GetTargetSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Calculate Target Slot"):
        super(GetTargetSlot, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        # คำนวณช่องเป้าหมายบนชั้นวาง
        cube_id = self.blackboard.get("detected_id")  # ดึง ID จาก Blackboard
        
        if cube_id is None:
            return py_trees.common.Status.FAILURE
            
        # Mapping Logic (กำหนดตำแหน่งตาม ID)
        # ID 1 -> (2, 1)
        # ID 2 -> (2, 2)
        # ID 3 -> (2, 3)
        # ID 4 -> (3, 1)
        # ID 5 -> (3, 2)
        # ID 6 -> (3, 3)
        
        row, col = 0, 0
        if cube_id == 1: row, col = 2, 1
        elif cube_id == 2: row, col = 2, 2
        elif cube_id == 3: row, col = 2, 3
        elif cube_id == 4: row, col = 3, 1
        elif cube_id == 5: row, col = 3, 2
        elif cube_id == 6: row, col = 3, 3
        else:
            self.node.get_logger().warn(f"Unknown ID: {cube_id}")
            return py_trees.common.Status.FAILURE
            
        self.blackboard.set("target_row", row)  # บันทึกแถวเป้าหมาย
        self.blackboard.set("target_col", col)  # บันทึกคอลัมน์เป้าหมาย
        self.node.get_logger().info(f"🎯 Target Slot: Row {row}, Col {col}")
        return py_trees.common.Status.SUCCESS

# --- Tree Construction (สร้างโครงสร้าง Tree) ---

def create_root():
    # ฟังก์ชันสร้าง Tree (ตัวอย่างเก่า ไม่ได้ใช้จริงใน main)
    root = py_trees.composites.Sequence(name="Cube Sorting Task", memory=True)
    
    # Blackboard
    blackboard = py_trees.blackboard.Blackboard()

    # 1. Move to Pickup Zone (Shelf 1, Col 2)
    move_pickup = py_trees_ros.actions.ActionClient(
        name="Go to Pickup Zone",
        action_type=MoveToShelf,
        action_name="move_to_shelf",
        action_goal=MoveToShelf.Goal(row=1, col=2)
    )

    # 2. Scan & Detect (Retry until found)
    scan_seq = py_trees.composites.Sequence(name="Scan Sequence", memory=True)
    scan_action = ScanForCube()
    # Retry scanning 10 times (ลองใหม่ 10 ครั้งถ้าไม่เจอ)
    scan_retry = py_trees.decorators.Retry(name="Retry Scan", child=scan_action, num_failures=10)
    scan_seq.add_child(scan_retry)

    # 3. Pick Sequence
    pick_seq = py_trees.composites.Sequence(name="Pick Sequence", memory=True)
    
    # ... (ส่วนนี้ยังไม่สมบูรณ์ในฟังก์ชันตัวอย่างนี้)
    
    return root

# --- Helper for Dynamic Actions (ตัวช่วยสำหรับ Action แบบ Dynamic) ---

class DynamicApproach(py_trees.behaviour.Behaviour):
    def __init__(self, name="Dynamic Approach"):
        super(DynamicApproach, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = rclpy.action.ActionClient(self.node, ApproachTag, 'approach_tag')
        self.logger = self.node.get_logger()

    def initialise(self):
        # เริ่มต้นก่อนทำงาน
        self.tag_name = self.blackboard.get("detected_tag_name")  # ดึงชื่อ Tag จาก Blackboard
        self.logger.info(f"Approaching {self.tag_name}")
        self.goal_sent = False
        self.future = None
        self.goal_handle = None
        self.result_future = None

    def update(self):
        # ส่ง Goal และรอผลลัพธ์
        if not self.goal_sent:
            goal = ApproachTag.Goal()
            goal.tag_name = self.tag_name
            goal.distance = 0.20
            goal.roll_offset = 0.0
            
            self.future = self.action_client.send_goal_async(goal)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
            
        if self.future and not self.future.done():
            return py_trees.common.Status.RUNNING
            
        if self.future and self.future.done() and not self.goal_handle:
            self.goal_handle = self.future.result()
            if not self.goal_handle.accepted:
                return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            
        if self.result_future and not self.result_future.done():
            return py_trees.common.Status.RUNNING
            
        if self.result_future and self.result_future.done():
            result = self.result_future.result().result
            return py_trees.common.Status.SUCCESS if result.success else py_trees.common.Status.FAILURE
            
        return py_trees.common.Status.FAILURE

class DynamicVisualServo(py_trees.behaviour.Behaviour):
    def __init__(self, name="Dynamic Visual Servo"):
        super(DynamicVisualServo, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = rclpy.action.ActionClient(self.node, VisualServo, 'visual_servo')

    def initialise(self):
        self.tag_name = self.blackboard.get("detected_tag_name")
        self.goal_sent = False
        self.future = None
        self.goal_handle = None
        self.result_future = None

    def update(self):
        if not self.goal_sent:
            goal = VisualServo.Goal()
            goal.tag_name = self.tag_name
            goal.tolerance = 0.01
            self.future = self.action_client.send_goal_async(goal)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
            
        if self.future and not self.future.done(): return py_trees.common.Status.RUNNING
        if self.future and self.future.done() and not self.goal_handle:
            self.goal_handle = self.future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            
        if self.result_future and not self.result_future.done(): return py_trees.common.Status.RUNNING
        if self.result_future and self.result_future.done():
            return py_trees.common.Status.SUCCESS if self.result_future.result().result.success else py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

class DynamicMoveToShelf(py_trees.behaviour.Behaviour):
    def __init__(self, name="Dynamic Move Shelf"):
        super(DynamicMoveToShelf, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.action_client = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.action_client = rclpy.action.ActionClient(self.node, MoveToShelf, 'move_to_shelf')

    def initialise(self):
        self.row = self.blackboard.get("target_row")
        self.col = self.blackboard.get("target_col")
        self.goal_sent = False
        self.future = None
        self.goal_handle = None
        self.result_future = None

    def update(self):
        if not self.goal_sent:
            goal = MoveToShelf.Goal()
            goal.row = self.row
            goal.col = self.col
            self.future = self.action_client.send_goal_async(goal)
            self.goal_sent = True
            return py_trees.common.Status.RUNNING
            
        if self.future and not self.future.done(): return py_trees.common.Status.RUNNING
        if self.future and self.future.done() and not self.goal_handle:
            self.goal_handle = self.future.result()
            if not self.goal_handle.accepted: return py_trees.common.Status.FAILURE
            self.result_future = self.goal_handle.get_result_async()
            
        if self.result_future and not self.result_future.done(): return py_trees.common.Status.RUNNING
        if self.result_future and self.result_future.done():
            return py_trees.common.Status.SUCCESS if self.result_future.result().result.success else py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE

# --- Re-Create Root with Dynamic Actions (สร้าง Tree เต็มรูปแบบ) ---

def create_full_tree():
    root = py_trees.composites.Sequence(name="Cube Sorting Task", memory=True)
    
    # 1. Move to Pickup Zone (ไปที่จุดรับของ)
    move_pickup = py_trees_ros.actions.ActionClient(
        name="Go to Pickup Zone",
        action_type=MoveToShelf,
        action_name="move_to_shelf",
        action_goal=MoveToShelf.Goal(row=1, col=2)
    )
    root.add_child(move_pickup)

    # 2. Scan (สแกนหาของ)
    scan = ScanForCube()
    scan_retry = py_trees.decorators.Retry(name="Retry Scan", child=scan, num_failures=20)
    root.add_child(scan_retry)

    # 3. Pick Sequence (ลำดับการหยิบ)
    pick_seq = py_trees.composites.Sequence(name="Pick Sequence", memory=True)
    
    approach = DynamicApproach()
    servo = DynamicVisualServo()
    
    gripper_open = py_trees_ros.actions.ActionClient(
        name="Open Gripper",
        action_type=GripperControl,
        action_name="gripper_control",
        action_goal=GripperControl.Goal(open=True)
    )
    
    move_in = py_trees_ros.actions.ActionClient(
        name="Move In",
        action_type=MoveLinear,
        action_name="move_linear",
        action_goal=MoveLinear.Goal(distance=0.15) # 15cm approach
    )
    
    gripper_close = py_trees_ros.actions.ActionClient(
        name="Close Gripper",
        action_type=GripperControl,
        action_name="gripper_control",
        action_goal=GripperControl.Goal(open=False)
    )
    
    move_out = py_trees_ros.actions.ActionClient(
        name="Move Out",
        action_type=MoveLinear,
        action_name="move_linear",
        action_goal=MoveLinear.Goal(distance=-0.15) # Retreat
    )
    
    pick_seq.add_children([approach, servo, gripper_open, move_in, gripper_close, move_out])
    root.add_child(pick_seq)

    # 4. Calculate Slot (คำนวณช่องวาง)
    calc_slot = GetTargetSlot()
    root.add_child(calc_slot)

    # 5. Place Sequence (ลำดับการวาง)
    place_seq = py_trees.composites.Sequence(name="Place Sequence", memory=True)
    
    move_target = DynamicMoveToShelf()
    
    move_in_place = py_trees_ros.actions.ActionClient(
        name="Move In Place",
        action_type=MoveLinear,
        action_name="move_linear",
        action_goal=MoveLinear.Goal(distance=0.15)
    )
    
    gripper_open_place = py_trees_ros.actions.ActionClient(
        name="Release Cube",
        action_type=GripperControl,
        action_name="gripper_control",
        action_goal=GripperControl.Goal(open=True)
    )
    
    move_out_place = py_trees_ros.actions.ActionClient(
        name="Retreat Place",
        action_type=MoveLinear,
        action_name="move_linear",
        action_goal=MoveLinear.Goal(distance=-0.15)
    )
    
    place_seq.add_children([move_target, move_in_place, gripper_open_place, move_out_place])
    root.add_child(place_seq)
    
    # 6. Return Home (Loop back to start)
    # เนื่องจาก root เป็น Sequence(memory=True) เมื่อจบแล้วจะส่งคืน SUCCESS
    # เพื่อให้วนลูป เราจะใช้ Decorator ใน main
    
    return root

def main():
    rclpy.init(args=None)
    
    # Wrap in infinite loop (วนลูปไม่รู้จบ)
    task_root = create_full_tree()
    
    # ใช้ Repeat Decorator เพื่อให้ทำงานซ้ำ
    final_root = py_trees.decorators.Repeat(
        name="Infinite Loop",
        child=task_root,
        num_success=-1 # Infinite
    )
    
    tree = py_trees_ros.trees.BehaviourTree(root=final_root, unicode_tree_debug=True)
    tree.setup(timeout=15.0)
    
    print("🚀 Cube Sorting BT Started")
    
    try:
        tree.tick_tock(period_ms=500.0)  # Tick ทุกๆ 500ms
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        tree.interrupt()
    except Exception as e:
        console.logerror(console.red + f"Error: {e}" + console.reset)
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
