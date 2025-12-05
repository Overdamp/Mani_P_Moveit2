#!/usr/bin/env python3
import rclpy
import py_trees
import py_trees_ros
import py_trees.console as console
import sys
import time

from mani_p_actions.action import MoveToShelf, ApproachTag, VisualServo, MoveLinear, GripperControl
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener

# --- Custom Behaviors ---

class ScanForCube(py_trees.behaviour.Behaviour):
    def __init__(self, name="Scan For Cube"):
        super(ScanForCube, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.tf_buffer = None
        self.tf_listener = None

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

    def update(self):
        # Check for tags ID 1-6
        for i in range(1, 7):
            tag_frame = f"tag36h11:{i}"
            try:
                # Check if transform exists
                if self.tf_buffer.can_transform('Base_link', tag_frame, rclpy.time.Time()):
                    self.blackboard.set("detected_id", i)
                    self.blackboard.set("detected_tag_name", tag_frame)
                    self.node.get_logger().info(f"📦 Found Cube ID: {i}")
                    return py_trees.common.Status.SUCCESS
            except Exception:
                pass
        
        self.node.get_logger().info("Scanning... No cube found.")
        return py_trees.common.Status.FAILURE

class GetTargetSlot(py_trees.behaviour.Behaviour):
    def __init__(self, name="Calculate Target Slot"):
        super(GetTargetSlot, self).__init__(name)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        cube_id = self.blackboard.get("detected_id")
        
        if cube_id is None:
            return py_trees.common.Status.FAILURE
            
        # Mapping Logic
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
            
        self.blackboard.set("target_row", row)
        self.blackboard.set("target_col", col)
        self.node.get_logger().info(f"🎯 Target Slot: Row {row}, Col {col}")
        return py_trees.common.Status.SUCCESS

# --- Tree Construction ---

def create_root():
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
    # Retry scanning 10 times
    scan_retry = py_trees.decorators.Retry(name="Retry Scan", child=scan_action, num_failures=10)
    scan_seq.add_child(scan_retry)

    # 3. Pick Sequence
    pick_seq = py_trees.composites.Sequence(name="Pick Sequence", memory=True)
    
    # 3.1 Approach (Dynamic Goal from Blackboard)
    # Note: py_trees_ros ActionClient doesn't support dynamic goals easily out of the box without custom code.
    # We need a custom behaviour that reads blackboard and sends action.
    # For simplicity, I will implement a wrapper class below.
    
    # 3.2 Visual Servo
    # 3.3 Open Gripper
    # 3.4 Move Linear (Approach)
    # 3.5 Close Gripper
    # 3.6 Move Linear (Retreat)
    
    # ... Wait, standard ActionClient takes a static goal. We need dynamic goals.
    # I'll implement a DynamicActionClient for this.
    
    return root

# --- Helper for Dynamic Actions ---

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
        self.tag_name = self.blackboard.get("detected_tag_name")
        self.logger.info(f"Approaching {self.tag_name}")
        self.goal_sent = False
        self.future = None
        self.goal_handle = None
        self.result_future = None

    def update(self):
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

# --- Re-Create Root with Dynamic Actions ---

def create_full_tree():
    root = py_trees.composites.Sequence(name="Cube Sorting Task", memory=True)
    
    # 1. Move to Pickup Zone
    move_pickup = py_trees_ros.actions.ActionClient(
        name="Go to Pickup Zone",
        action_type=MoveToShelf,
        action_name="move_to_shelf",
        action_goal=MoveToShelf.Goal(row=1, col=2)
    )
    root.add_child(move_pickup)

    # 2. Scan
    scan = ScanForCube()
    scan_retry = py_trees.decorators.Retry(name="Retry Scan", child=scan, num_failures=20)
    root.add_child(scan_retry)

    # 3. Pick Sequence
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

    # 4. Calculate Slot
    calc_slot = GetTargetSlot()
    root.add_child(calc_slot)

    # 5. Place Sequence
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
    # Since root is a Sequence with memory=True, once it finishes, it returns SUCCESS.
    # To loop, we can wrap the whole thing in a decorator or just let the main loop restart it?
    # Usually, we wrap in a loop decorator.
    
    return root

def main():
    rclpy.init(args=None)
    
    # Wrap in infinite loop
    task_root = create_full_tree()
    # We want to clear memory after each full run so it restarts
    # But Sequence(memory=True) keeps state. 
    # Actually, if the Sequence finishes (SUCCESS), it resets itself for the next tick if we tick it again?
    # No, with memory=True, if it succeeds, it stays succeeded until reset.
    # We should use a Decorator to repeat.
    
    main_root = py_trees.decorators.OneShot(
        name="One Shot Task",
        child=task_root,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION
    )
    # Wait, user wants a loop.
    # Let's just use a standard Sequence without memory at the top level?
    # No, we need memory for the steps.
    
    # Correct way: Root is a Sequence(memory=True).
    # We wrap it in a loop?
    
    # Let's just make the root a Sequence(memory=True) and manually reset it in the main loop if it succeeds?
    # Or use py_trees.decorators.Repeat
    
    final_root = py_trees.decorators.Repeat(
        name="Infinite Loop",
        child=task_root,
        num_success=-1 # Infinite
    )
    
    tree = py_trees_ros.trees.BehaviourTree(root=final_root, unicode_tree_debug=True)
    tree.setup(timeout=15.0)
    
    print("🚀 Cube Sorting BT Started")
    
    try:
        tree.tick_tock(period_ms=500.0)
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        tree.interrupt()
    except Exception as e:
        console.logerror(console.red + f"Error: {e}" + console.reset)
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
