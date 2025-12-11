#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

class SimpleShelfMover(Node):
    def __init__(self):
        super().__init__('simple_shelf_mover')
        
        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            'shelf_goal_pose',
            self.goal_callback,
            10)
            
        # MoveIt Action Client
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info("🤖 Simple Shelf Mover Waiting for Goal on /shelf_goal_pose ...")

    def goal_callback(self, msg):
        self.get_logger().info("📩 Received Goal! Planning move...")
        self.get_logger().info(f"   Position: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})")
        self.get_logger().info(f"   Orientation: ({msg.pose.orientation.x:.3f}, {msg.pose.orientation.y:.3f}, {msg.pose.orientation.z:.3f}, {msg.pose.orientation.w:.3f})")
        self.send_move_goal(msg)

    def send_move_goal(self, pose_stamped):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ MoveGroup Action Server not available!")
            return

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
        
        # Box Constraint (Relaxed to 5cm)
        pcm.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 0.05, 0.05]))
        pcm.constraint_region.primitive_poses.append(pose_stamped.pose)
        pcm.weight = 1.0
        
        # Orientation Constraint (Relaxed)
        ocm = OrientationConstraint()
        ocm.header.frame_id = 'Base_link'
        ocm.link_name = 'tcp_link'
        ocm.orientation = pose_stamped.pose.orientation
        ocm.absolute_x_axis_tolerance = 0.5
        ocm.absolute_y_axis_tolerance = 0.5
        ocm.absolute_z_axis_tolerance = 0.5
        ocm.weight = 1.0
        
        goal_msg.request.goal_constraints.append(Constraints(position_constraints=[pcm], orientation_constraints=[ocm]))
        
        self.get_logger().info("🚀 Sending Goal to MoveIt...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by MoveIt')
            return

        self.get_logger().info('✅ Goal accepted! Moving...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1:
            self.get_logger().info('🎉 Move Succeeded!')
        else:
            self.get_logger().error(f'❌ Move Failed with error code: {result.error_code.val}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleShelfMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
