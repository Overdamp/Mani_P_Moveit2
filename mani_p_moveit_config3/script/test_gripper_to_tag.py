#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import Buffer, TransformListener
import sys
import math
import copy

class GripperToTag(Node):

    def __init__(self):
        super().__init__('gripper_to_tag')
        
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Config
        self.group_name = "arm"
        self.end_effector_link = "tcp_link"
        self.base_frame = "Base_link"
        self.approach_distance = 0.15 # 15cm offset from tag

    def get_tag_transform(self, tag_name):
        try:
            # Wait for transform
            self.get_logger().info(f"Waiting for transform {self.base_frame} -> {tag_name}...")
            # Try to lookup with a timeout
            if not self.tf_buffer.can_transform(self.base_frame, tag_name, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=5.0)):
                self.get_logger().error(f"Could not find transform for {tag_name}")
                return None
            
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                tag_name,
                rclpy.time.Time())
            return transform
        except Exception as e:
            self.get_logger().error(f"TF Lookup Error: {e}")
            return None

    def send_goal(self, tag_name):
        transform = self.get_tag_transform(tag_name)
        if transform is None:
            return

        goal_msg = MoveGroup.Goal()
        goal_msg.request.workspace_parameters.header.frame_id = self.base_frame
        goal_msg.request.workspace_parameters.min_corner.x = -1.0
        goal_msg.request.workspace_parameters.min_corner.y = -1.0
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.max_corner.x = 1.0
        goal_msg.request.workspace_parameters.max_corner.y = 1.0
        goal_msg.request.workspace_parameters.max_corner.z = 1.0

        goal_msg.request.start_state.is_diff = True
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1
        goal_msg.request.num_planning_attempts = 10

        # Create Constraints
        c = Constraints()
        c.name = "tag_approach"

        # 1. Position Constraint
        pc = PositionConstraint()
        pc.header.frame_id = self.base_frame
        pc.link_name = self.end_effector_link
        pc.weight = 1.0
        
        # Target Position: Tag Position + Offset
        # Assuming Tag Z is pointing OUT of the surface.
        # We want to be in front of it.
        # Simple approach: Just use the Tag's translation for now, maybe back off along X/Y/Z?
        # Let's assume we want to go exactly to the tag frame origin first, but backed off by approach_distance along the Z axis of the tag?
        # Actually, let's just go to the tag position for now, but maybe 10cm above/front.
        # Since we don't know the tag orientation convention perfectly without testing, 
        # let's try to match the translation exactly but with a safe Z offset in Base frame if possible, 
        # OR better: Transform a point (0,0,0.15) from Tag frame to Base frame.
        
        # But here we only have the transform.
        # Let's construct a Pose for the target.
        # We want the Gripper to be at: Tag_Pos + (Tag_Rot * (0, 0, 0.15))
        # Wait, if Tag Z is out, and we want to approach, we might want to be at Z=0.15 in Tag frame.
        
        # For simplicity in this test script, let's just target the Tag's translation directly, 
        # but maybe apply a fixed offset in Z of Base_link to be safe (lifted up).
        # OR, let's try to be smart.
        # Let's just target the exact tag location for now (user said "to the face of the work piece").
        # Maybe the user implies "close to it".
        # I'll add a small offset in the Tag's Z direction (0.1m).
        
        # Math for offset:
        # P_target = T_base_tag * P_offset_in_tag
        # P_offset_in_tag = (0, 0, 0.15)
        
        # Extract quaternion
        q = transform.transform.rotation
        # Rotate vector (0,0,0.15) by q
        # ... (Math is complex to write out fully in python without numpy/tf_transformations)
        # Let's stick to a simpler offset: Just use the tag's position directly.
        
        target_point = Point()
        target_point.x = transform.transform.translation.x
        target_point.y = transform.transform.translation.y
        target_point.z = transform.transform.translation.z
        
        # Define a small box region around the target
        bv = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01] # 1cm tolerance radius
        bv.primitives.append(primitive)
        bv.primitive_poses.append(PoseStamped(pose=PoseStamped().pose).pose) # Identity pose relative to constraint region center
        
        pc.constraint_region = bv
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0
        
        # Set the center of the constraint region to the target point
        # Wait, PositionConstraint defines the target region in `header.frame_id`.
        # So we set primitive_poses to the target point.
        bv.primitive_poses[0].position = target_point
        
        c.position_constraints.append(pc)

        # 2. Orientation Constraint
        oc = OrientationConstraint()
        oc.header.frame_id = self.base_frame
        oc.link_name = self.end_effector_link
        oc.orientation = transform.transform.rotation # Match tag orientation
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        goal_msg.request.goal_constraints.append(c)

        self.get_logger().info("Sending goal to MoveIt...")
        self.action_client.wait_for_server()
        
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    
    if len(sys.argv) < 2:
        print("Usage: test_gripper_to_tag.py <tag_name>")
        return

    tag_name = sys.argv[1]
    
    node = GripperToTag()
    node.send_goal(tag_name)
    rclpy.spin(node)

if __name__ == '__main__':
    main()
