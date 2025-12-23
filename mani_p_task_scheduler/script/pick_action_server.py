#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mani_p_actions.action import MoveToShelf, GripperControl, MoveLinear

class PickActionServer(Node):

    def __init__(self):
        super().__init__('pick_action_server')
        
        self.cb_group = ReentrantCallbackGroup()

        # Action Server: Pick From Shelf (Uses MoveToShelf message type for convenience)
        self._action_server = ActionServer(
            self,
            MoveToShelf,
            'pick_from_shelf',
            self.execute_callback,
            callback_group=self.cb_group
        )
        
        # Action Clients
        self._shelf_client = ActionClient(self, MoveToShelf, 'move_to_shelf', callback_group=self.cb_group)
        self._gripper_client = ActionClient(self, GripperControl, 'gripper_control', callback_group=self.cb_group)
        self._linear_client = ActionClient(self, MoveLinear, 'move_linear', callback_group=self.cb_group)
        
        # Shelf Map (Row, Col) -> Cube ID
        self.shelf_map = {
            (1, 1): "cube1", (1, 2): "cube2", (1, 3): "cube3",
            (2, 1): "cube4", (2, 2): "cube5", (2, 3): "cube6",
            (3, 1): "cube7", (3, 2): "cube8", (3, 3): "cube9"
        }
        
        # Parameters
        self.standoff_dist = 0.10 # 10cm approach/retreat
        self.lift_dist = 0.05     # 5cm lift up
        
        self.get_logger().info('✅ Pick Action Server Ready.')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Pick Sequence...')
        
        row = goal_handle.request.row
        col = goal_handle.request.col
        cube_id = self.shelf_map.get((row, col), "cube1")
        
        result = MoveToShelf.Result()
        
        # 1. Move to Shelf (Standoff Position)
        self.get_logger().info(f"1. Moving to Shelf Slot ({row}, {col})...")
        if not await self.call_shelf_action(row, col):
            result.success = False
            result.message = "Failed to move to shelf."
            goal_handle.abort()
            return result
            
        # 2. Open Gripper
        self.get_logger().info("2. Opening Gripper...")
        if not await self.call_gripper(open=True, target_id=cube_id):
            result.success = False
            result.message = "Failed to open gripper."
            goal_handle.abort()
            return result
            
        # 3. Approach (Move Forward)
        self.get_logger().info(f"3. Approaching ({self.standoff_dist}m)...")
        if not await self.call_linear(direction="forward", distance=self.standoff_dist):
            result.success = False
            result.message = "Failed to approach."
            goal_handle.abort()
            return result
            
        # 4. Grasp (Close Gripper)
        self.get_logger().info(f"4. Grasping {cube_id}...")
        if not await self.call_gripper(open=False, target_id=cube_id):
            result.success = False
            result.message = "Failed to grasp."
            goal_handle.abort()
            return result
            
        # 5. Lift Up (New Step)
        self.get_logger().info(f"5. Lifting Up ({self.lift_dist}m)...")
        if not await self.call_linear(direction="up", distance=self.lift_dist):
            result.success = False
            result.message = "Failed to lift up."
            goal_handle.abort()
            return result
            
        # 6. Retreat (Move Backward)
        self.get_logger().info(f"6. Retreating ({self.standoff_dist}m)...")
        if not await self.call_linear(direction="backward", distance=self.standoff_dist):
            result.success = False
            result.message = "Failed to retreat."
            goal_handle.abort()
            return result
            
        self.get_logger().info("✅ Pick Sequence Complete!")
        result.success = True
        result.message = "Object Picked Successfully!"
        goal_handle.succeed()
        return result

    async def call_shelf_action(self, row, col):
        if not self._shelf_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Shelf Action Server not available")
            return False
            
        goal = MoveToShelf.Goal()
        goal.row = row
        goal.col = col
        
        future = self._shelf_client.send_goal_async(goal)
        while not future.done(): 
            import time
            time.sleep(0.01)
        goal_handle = future.result()
        
        if not goal_handle.accepted: return False
        
        res_future = goal_handle.get_result_async()
        while not res_future.done(): 
            import time
            time.sleep(0.1)
        return res_future.result().result.success

    async def call_gripper(self, open, target_id):
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper Action Server not available")
            return False

        goal = GripperControl.Goal()
        goal.open = open
        goal.target_id = target_id
        
        future = self._gripper_client.send_goal_async(goal)
        while not future.done(): 
            import time
            time.sleep(0.01)
        goal_handle = future.result()
        
        if not goal_handle.accepted: return False
        
        res_future = goal_handle.get_result_async()
        while not res_future.done(): 
            import time
            time.sleep(0.1)
        return res_future.result().result.success

    async def call_linear(self, direction, distance):
        if not self._linear_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Move Linear Action Server not available")
            return False

        goal = MoveLinear.Goal()
        goal.direction = direction
        goal.distance = distance
        
        future = self._linear_client.send_goal_async(goal)
        while not future.done(): 
            import time
            time.sleep(0.01)
        goal_handle = future.result()
        
        if not goal_handle.accepted: return False
        
        res_future = goal_handle.get_result_async()
        while not res_future.done(): 
            import time
            time.sleep(0.1)
        return res_future.result().result.success

def main(args=None):
    rclpy.init(args=args)
    node = PickActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
