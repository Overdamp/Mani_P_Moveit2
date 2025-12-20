#!/usr/bin/env python3
import rclpy  # นำเข้าไลบรารี rclpy
from rclpy.node import Node  # นำเข้าคลาส Node
from rclpy.action import ActionClient  # นำเข้า ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup  # นำเข้า Callback Group แบบ Reentrant
import sys  # นำเข้า sys
import time  # นำเข้า time

# Custom Actions (นำเข้า Action ที่สร้างเอง)
from mani_p_actions.action import ApproachTag, AdjustLevel

class ActionScheduler(Node):

    def __init__(self):
        super().__init__('action_scheduler')  # สร้าง Node ชื่อ 'action_scheduler'
        
        self.cb_group = ReentrantCallbackGroup()  # สร้าง Callback Group เพื่อให้ทำงานขนานกันได้
        
        # Clients (สร้าง Action Client)
        self.approach_client = ActionClient(self, ApproachTag, 'approach_tag', callback_group=self.cb_group)
        self.adjust_level_client = ActionClient(self, AdjustLevel, 'adjust_level', callback_group=self.cb_group)
        
        self.get_logger().info('✅ Action Scheduler Ready (No Visual Servo).')

    def run_mission(self, tag_name="tag2"):
        # ฟังก์ชันหลักสำหรับรันภารกิจ
        self.get_logger().info(f"🚀 Starting Mission for {tag_name}")
        
        # 1. Approach (เข้าหา Tag)
        if not self.call_approach(tag_name, 0.25, 0.0):
            self.get_logger().error("❌ Mission Failed at Approach Step")
            return

        # 2. Adjust Level (Skip Align) (ปรับระนาบ)
        if not self.call_adjust_level(0.0):
            self.get_logger().error("❌ Mission Failed at Level Step")
            return
            
        self.get_logger().info("🎉 MISSION COMPLETE! Robot is aligned and level.")

    def call_approach(self, tag, dist, roll):
        # เรียกใช้ Action ApproachTag
        self.get_logger().info("🔵 Step 1: Approach Tag...")
        
        if not self.approach_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Approach Server not available")
            return False
            
        goal = ApproachTag.Goal()
        goal.tag_name = tag
        goal.distance = dist
        goal.roll_offset = roll
        
        future = self.approach_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.1)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return False
            
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)
            
        return res_future.result().result.success

    def call_adjust_level(self, roll):
        # เรียกใช้ Action AdjustLevel
        self.get_logger().info("🔵 Step 2: Adjust Level...")
        
        if not self.adjust_level_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Adjust Level Server not available")
            return False
            
        goal = AdjustLevel.Goal()
        goal.target_roll = roll
        
        future = self.adjust_level_client.send_goal_async(goal)
        while not future.done():
            time.sleep(0.1)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            return False
            
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)
            
        return res_future.result().result.success

def main(args=None):
    rclpy.init(args=args)  # เริ่มต้น ROS 2
    
    tag = "tag2"
    if len(sys.argv) > 1:
        tag = sys.argv[1]
        
    scheduler = ActionScheduler()
    
    import threading
    # รัน spin ใน Thread แยก เพื่อให้ Action Client ทำงานได้ในขณะที่ Main Thread รอผลลัพธ์
    spin_thread = threading.Thread(target=rclpy.spin, args=(scheduler,), daemon=True)
    spin_thread.start()
    
    try:
        scheduler.run_mission(tag)
    except KeyboardInterrupt:
        pass
        
    scheduler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
