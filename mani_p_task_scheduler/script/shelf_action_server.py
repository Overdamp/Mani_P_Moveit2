#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mani_p_actions.action import MoveToShelf
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

# MoveIt
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from rclpy.action import ActionClient

class ShelfActionServer(Node):
    def __init__(self):
        super().__init__('shelf_action_server')
        
        self.callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            MoveToShelf,
            'move_to_shelf',
            self.execute_callback,
            callback_group=self.callback_group)
            
        # TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # MoveIt Action Client
        self._move_group_client = ActionClient(self, MoveGroup, 'move_action', callback_group=self.callback_group)
        
        # Shelf Config
        self.ref_tag = "tag36h11:11" # Top Center Tag
        self.row_spacing = 0.40
        self.col_spacing = 0.40
        self.standoff_dist = 0.25
        
        self.get_logger().info("📦 Shelf Action Server Ready")

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        feedback_msg = MoveToShelf.Feedback()
        result = MoveToShelf.Result()
        
        row = goal_handle.request.row
        col = goal_handle.request.col
        
        feedback_msg.status = f"Calculating target for Row {row}, Col {col}..."
        goal_handle.publish_feedback(feedback_msg)
        
        # 1. Calculate Target Pose
        target_pose = self.get_target_pose(row, col)
        
        if not target_pose:
            result.success = False
            result.message = "Failed to calculate target pose (Tag not visible?)"
            goal_handle.abort()
            return result
            
        feedback_msg.status = "Planning path..."
        goal_handle.publish_feedback(feedback_msg)
        
        # 2. Execute Move
        success = await self.move_to_pose(target_pose)
        
        if success:
            result.success = True
            result.message = "Arrived at shelf slot."
            goal_handle.succeed()
        else:
            result.success = False
            result.message = "Motion planning failed."
            goal_handle.abort()
            
        return result

    def get_target_pose(self, row, col):
        # ... (Logic from shelf_navigator.py) ...
        y_offset = row * self.row_spacing
        
        if col == 1: x_offset = -self.col_spacing
        elif col == 2: x_offset = 0.0
        elif col == 3: x_offset = self.col_spacing
        else: return None
        
        z_offset = self.standoff_dist
        
        target_pose_tag = PoseStamped()
        target_pose_tag.header.frame_id = self.ref_tag
        target_pose_tag.header.stamp = self.get_clock().now().to_msg()
        target_pose_tag.pose.position.x = x_offset
        target_pose_tag.pose.position.y = y_offset
        target_pose_tag.pose.position.z = z_offset
        target_pose_tag.pose.orientation.w = 1.0 # Identity (Face same way as tag)
        
        try:
            if not self.tf_buffer.can_transform('Base_link', self.ref_tag, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
                return None
            transform = self.tf_buffer.lookup_transform('Base_link', self.ref_tag, rclpy.time.Time())
            target_pose_base = tf2_geometry_msgs.do_transform_pose(target_pose_tag.pose, transform)
            
            result = PoseStamped()
            result.header.frame_id = 'Base_link'
            result.pose = target_pose_base
            return result
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return None

    async def move_to_pose(self, pose_stamped):
        if not self._move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available')
            return False

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # Position Constraint
        pcm = PositionConstraint()
        pcm.header.frame_id = 'Base_link'
        pcm.link_name = 'tcp_link'
        pcm.target_point_offset.x = 0.0
        pcm.target_point_offset.y = 0.0
        pcm.target_point_offset.z = 0.0
        
        # Create a small box for the target position
        from shape_msgs.msg import SolidPrimitive
        pcm.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.01, 0.01, 0.01]))
        pcm.constraint_region.primitive_poses.append(pose_stamped.pose)
        pcm.weight = 1.0
        
        # Orientation Constraint (Keep Level)
        ocm = OrientationConstraint()
        ocm.header.frame_id = 'Base_link'
        ocm.link_name = 'tcp_link'
        ocm.orientation = pose_stamped.pose.orientation # Use calculated orientation
        ocm.absolute_x_axis_tolerance = 0.5
        ocm.absolute_y_axis_tolerance = 0.5
        ocm.absolute_z_axis_tolerance = 0.5
        ocm.weight = 1.0
        
        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
        
        self.get_logger().info("Sending MoveGroup goal...")
        send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False
            
        result_future = goal_handle.get_result_async()
        result = await result_future
        
        if result.result.error_code.val == 1: # SUCCESS
            return True
        else:
            self.get_logger().error(f"MoveIt Error Code: {result.result.error_code.val}")
            return False

def main(args=None):
    rclpy.init(args=args)
    node = ShelfActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
