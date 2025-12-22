#!/usr/bin/env python3
# ระบุว่าไฟล์นี้เป็น Python script ที่รันด้วย python3

import rclpy
# นำเข้าไลบรารี rclpy สำหรับการเขียน ROS 2 Node
from rclpy.node import Node
# นำเข้าคลาส Node เพื่อสร้าง ROS 2 Node
from rclpy.action import ActionClient
# นำเข้า ActionClient สำหรับเรียกใช้ Action Server
from tf2_ros import Buffer, TransformListener
# นำเข้า Buffer และ TransformListener สำหรับการจัดการ TF (Transform)
from geometry_msgs.msg import TransformStamped
# นำเข้า message type TransformStamped
import csv
# นำเข้าไลบรารี csv สำหรับการบันทึกไฟล์ CSV
import math
# นำเข้าไลบรารี math สำหรับการคำนวณทางคณิตศาสตร์
import time
# นำเข้าไลบรารี time สำหรับจัดการเวลา
import os
# นำเข้าไลบรารี os สำหรับจัดการไฟล์และ path
from datetime import datetime
# นำเข้า datetime เพื่อดึงเวลาปัจจุบัน

from mani_p_actions.action import MoveToShelf
# นำเข้า Action Definition

class GraspValidator(Node):
    # สร้างคลาส GraspValidator โดยสืบทอดมาจาก Node ของ ROS 2

    def __init__(self):
        # ฟังก์ชันเริ่มต้น (Constructor) ของคลาส
        super().__init__('grasp_validator')
        # เรียกใช้ Constructor ของคลาสแม่ (Node) และตั้งชื่อ Node ว่า 'grasp_validator'

        # ==========================================
        # 🛠️ การตั้งค่า (Configuration) 🛠️
        # ==========================================
        
        self.tf_buffer = Buffer()
        # สร้าง Buffer สำหรับเก็บข้อมูล TF
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # สร้าง Listener เพื่อรอรับข้อมูล TF และเก็บลง Buffer

        self.csv_filename = 'grasp_accuracy_log.csv'
        # กำหนดชื่อไฟล์ CSV ที่จะบันทึกผลลัพธ์
        
        # Updated to 0.10m to match Shelf Action Server Standoff
        self.ideal_grasp_depth = 0.10
        # กำหนดค่าความลึกที่เหมาะสมในการจับ (Ideal Grasp Depth)
        # หมายความว่า TCP ควรจะอยู่ห่างจาก Tag 10 ซม. (Standoff)

        # สร้าง Dictionary สำหรับจับคู่ (Row, Col) ไปยัง Tag ID
        self.shelf_map = {
            (1, 1): 1, (1, 2): 2, (1, 3): 3,
            (2, 1): 4, (2, 2): 5, (2, 3): 6,
            (3, 1): 7, (3, 2): 8, (3, 3): 9
        }
        # จบการสร้าง Dictionary

        # Action Client Setup
        self._action_client = ActionClient(self, MoveToShelf, 'move_to_shelf')
        
        self.init_csv()
        # เรียกฟังก์ชันเพื่อเตรียมไฟล์ CSV (สร้างหัวตารางถ้ายังไม่มีไฟล์)

        self.get_logger().info('Grasp Validator Node Started.')
        # แสดงข้อความแจ้งเตือนว่า Node เริ่มทำงานแล้ว
        self.get_logger().info(f'Logging to: {os.path.abspath(self.csv_filename)}')
        # แสดง path ของไฟล์ CSV ที่จะบันทึก

        self.run_loop()
        # เรียกฟังก์ชันหลักในการทำงานวนลูปรับค่าจากผู้ใช้

    def init_csv(self):
        # ฟังก์ชันสำหรับเตรียมไฟล์ CSV
        if not os.path.exists(self.csv_filename):
            # ตรวจสอบว่าไฟล์ CSV ยังไม่มีอยู่หรือไม่
            with open(self.csv_filename, mode='w', newline='') as file:
                # ถ้ายังไม่มี ให้สร้างไฟล์ใหม่ในโหมดเขียน ('w')
                writer = csv.writer(file)
                # สร้าง object writer
                writer.writerow(['Timestamp', 'Target_Row', 'Target_Col', 'Tag_ID', 'Error_X', 'Error_Y', 'Error_Z', 'Total_Euclidean_Error', 'Note'])
                # เขียนหัวตาราง (Header) ลงไปในไฟล์

    def get_user_input(self):
        # ฟังก์ชันสำหรับรับค่าจากผู้ใช้
        while True:
            # วนลูปจนกว่าจะได้ค่าที่ถูกต้อง
            try:
                print("\n========================================")
                # พิมพ์เส้นคั่นเพื่อความสวยงาม
                user_in = input("Enter Target Slot (Row Col) e.g., '1 2' or 'q' to quit: ")
                # รับค่าจากผู้ใช้ ให้ใส่ Row และ Col เว้นวรรคกัน หรือกด q เพื่อออก
                
                if user_in.lower() == 'q':
                    # ถ้าผู้ใช้พิมพ์ 'q'
                    return None, None
                    # คืนค่า None เพื่อจบการทำงาน

                parts = user_in.split()
                # แยกข้อความด้วยช่องว่าง
                if len(parts) != 2:
                    # ถ้าไม่ได้ใส่มา 2 ค่า
                    print("Invalid input format. Please enter Row and Col separated by space.")
                    # แจ้งเตือนว่ารูปแบบผิด
                    continue
                    # วนลูปใหม่

                row = int(parts[0])
                # แปลงค่าแรกเป็นจำนวนเต็ม (Row)
                col = int(parts[1])
                # แปลงค่าที่สองเป็นจำนวนเต็ม (Col)

                if (row, col) in self.shelf_map:
                    # ตรวจสอบว่า (Row, Col) ที่ใส่มา มีอยู่ใน Map หรือไม่
                    return row, col
                    # ถ้ามี ให้คืนค่า Row และ Col กลับไป
                else:
                    print("Slot not found. Row and Col must be 1-3.")
                    # ถ้าไม่มี แจ้งเตือนว่าไม่พบ Slot นี้

            except ValueError:
                # ดักจับ Error กรณีที่ใส่ค่าไม่ใช่ตัวเลข
                print("Invalid input. Please enter numbers.")
                # แจ้งเตือนให้ใส่ตัวเลข

    def measure_error(self, tag_id):
        # ฟังก์ชันสำหรับวัดค่าความคลาดเคลื่อน (Error)
        target_frame = f'tag{tag_id}_fisheye'
        # สร้างชื่อ Frame ของ Tag เป้าหมาย เช่น 'tag1_fisheye' (ตามที่ตั้งใน config)
        source_frame = 'tcp_link'
        # กำหนด Frame ต้นทางคือ 'tcp_link' (ปลายมือจับของหุ่นยนต์)

        try:
            # พยายามดึงค่า Transform
            # เราต้องการหาตำแหน่งของ tcp_link เทียบกับ tag (tag เป็นจุดอ้างอิง 0,0,0)
            # ดังนั้น from_frame = tag, to_frame = tcp_link
            t = self.tf_buffer.lookup_transform(
                target_frame, # Frame อ้างอิง (Target Tag)
                source_frame, # Frame ที่ต้องการรู้ตำแหน่ง (TCP)
                rclpy.time.Time() # เวลาปัจจุบัน (ล่าสุดที่มี)
            )
            
            # ดึงค่าตำแหน่ง X, Y, Z ที่วัดได้
            measured_x = t.transform.translation.x
            measured_y = t.transform.translation.y
            measured_z = t.transform.translation.z

            # คำนวณ Error
            # Error X คือระยะห่างในแนวแกน X (Lateral)
            error_x = measured_x
            
            # Error Y คือระยะห่างในแนวแกน Y (Vertical)
            error_y = measured_y
            
            # Error Z คือระยะห่างในแนวแกน Z (Depth) ลบด้วย Ideal Depth
            # เพราะเราต้องการให้ TCP จมเข้าไปใน Tag ตามระยะ Ideal
            # ถ้า TCP อยู่ที่ Z=0.10 (พอดีเป๊ะ) -> Error = 0.10 - 0.10 = 0
            error_z = measured_z - self.ideal_grasp_depth

            # คำนวณ Euclidean Error (ระยะห่างรวมแบบ 3 มิติ)
            euclidean_error = math.sqrt(error_x**2 + error_y**2 + error_z**2)

            return error_x, error_y, error_z, euclidean_error
            # คืนค่า Error ทั้งหมดกลับไป

        except Exception as e:
            # ดักจับ Error กรณีหา TF ไม่เจอ
            self.get_logger().warn(f"Could not get transform from {target_frame} to {source_frame}: {e}")
            # แสดงข้อความแจ้งเตือน
            return None
            # คืนค่า None เพื่อบอกว่าวัดค่าไม่ได้

    def send_goal(self, row, col):
        # ฟังก์ชันส่ง Goal ไปยัง Action Server
        self.get_logger().info(f'Waiting for action server...')
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False

        goal_msg = MoveToShelf.Goal()
        goal_msg.row = row
        goal_msg.col = col
        
        self.get_logger().info(f'Sending goal: Row={row}, Col={col}')
        
        # ส่ง Goal แบบ Synchronous (รอจนกว่าจะเสร็จใน Loop นี้)
        # หมายเหตุ: ใน ROS 2 การเรียกแบบ sync ใน spin thread เดียวกันอาจ deadlock ได้
        # แต่ในที่นี้เราแยก thread spin ไว้แล้ว หรือเราจะใช้ send_goal_async แล้วรอ future ก็ได้
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # รอให้ Server รับ Goal
        while not send_goal_future.done():
            time.sleep(0.1)
            
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return False

        self.get_logger().info('Goal accepted, moving...')
        
        get_result_future = goal_handle.get_result_async()
        
        # รอจนกว่าจะทำงานเสร็จ
        while not get_result_future.done():
            time.sleep(0.1)
            
        result = get_result_future.result().result
        
        if result.success:
            self.get_logger().info(f'Movement Success: {result.message}')
            return True
        else:
            self.get_logger().error(f'Movement Failed: {result.message}')
            return False

    def run_loop(self):
        # ฟังก์ชันหลักในการวนลูปทำงาน
        print("\nSelect Mode:")
        print("1. Auto Move & Measure (Robot will move!)")
        print("2. Measure Only (Manual Jog)")
        mode_in = input("Select (1/2): ")
        
        auto_move = (mode_in.strip() == '1')
        
        if auto_move:
            print(">>> MODE: AUTO MOVE & MEASURE <<<")
        else:
            print(">>> MODE: MEASURE ONLY <<<")

        while rclpy.ok():
            # วนลูปตราบเท่าที่ ROS ยังทำงานอยู่
            row, col = self.get_user_input()
            # เรียกฟังก์ชันรับค่าจากผู้ใช้
            
            if row is None:
                # ถ้าผู้ใช้กด 'q' (คืนค่า None)
                print("Exiting...")
                # แสดงข้อความออกจากโปรแกรม
                break
                # จบลูป

            tag_id = self.shelf_map[(row, col)]
            # หา Tag ID จาก Row, Col ที่ได้มา
            print(f"Target: Row {row}, Col {col} -> Tag ID: {tag_id}")
            # แสดงข้อมูลเป้าหมาย

            if auto_move:
                print("Moving robot...")
                success = self.send_goal(row, col)
                if not success:
                    print("Skipping measurement due to movement failure.")
                    continue
                
                # รอสักนิดให้หุ่นนิ่งสนิท
                time.sleep(1.0)
            else:
                input(f"Press Enter when robot is at Slot ({row}, {col})...")
                # รอให้ผู้ใช้กด Enter เมื่อหุ่นยนต์เคลื่อนที่ไปถึงจุดแล้ว

            print("Measuring...")
            # แสดงข้อความกำลังวัดค่า
            
            # ลองวัดค่า (Retry ได้ถ้าไม่เจอ)
            result = self.measure_error(tag_id)
            
            if result:
                # ถ้าวัดค่าสำเร็จ
                err_x, err_y, err_z, euc_err = result
                # แตกค่าผลลัพธ์ออกมาเก็บในตัวแปร
                
                print(f"Result for Tag {tag_id}:")
                # แสดงผลลัพธ์
                print(f"  Error X: {err_x:.4f} m")
                print(f"  Error Y: {err_y:.4f} m")
                print(f"  Error Z: {err_z:.4f} m")
                print(f"  Euclidean Error: {euc_err:.4f} m")

                # บันทึกลง CSV
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                # สร้าง Timestamp ปัจจุบัน
                with open(self.csv_filename, mode='a', newline='') as file:
                    # เปิดไฟล์ CSV ในโหมดเพิ่มข้อมูลต่อท้าย ('a')
                    writer = csv.writer(file)
                    # สร้าง object writer
                    writer.writerow([timestamp, row, col, tag_id, f"{err_x:.4f}", f"{err_y:.4f}", f"{err_z:.4f}", f"{euc_err:.4f}", "Auto" if auto_move else "Manual"])
                    # เขียนข้อมูลลงไปในแถวใหม่
                
                print(f"Data saved to {self.csv_filename}")
                # แจ้งเตือนว่าบันทึกข้อมูลแล้ว
            else:
                # ถ้าวัดค่าไม่สำเร็จ
                print("Failed to measure error. Please check if camera can see the tag.")
                # แจ้งเตือนให้ตรวจสอบกล้อง

def main(args=None):
    # ฟังก์ชันหลักของโปรแกรม
    rclpy.init(args=args)
    # เริ่มต้นระบบ ROS 2
    node = GraspValidator()
    # สร้าง Node GraspValidator
    
    # แยก Thread สำหรับ spin ROS
    import threading
    
    executor = rclpy.executors.MultiThreadedExecutor()
    # สร้าง Executor แบบ Multi-thread
    executor.add_node(node)
    # เพิ่ม Node เข้าไปใน Executor
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    # สร้าง Thread ใหม่เพื่อรัน executor.spin()
    spin_thread.start()
    # เริ่มต้น Thread
    
    try:
        # รอให้ Thread ทำงานไปเรื่อยๆ (Main thread จะติดอยู่ที่ run_loop ของ node)
        spin_thread.join()
    except KeyboardInterrupt:
        # ถ้ากด Ctrl+C
        pass
    finally:
        node.destroy_node()
        # ทำลาย Node
        rclpy.shutdown()
        # ปิดระบบ ROS 2

if __name__ == '__main__':
    # ตรวจสอบว่าเป็นไฟล์หลักหรือไม่
    main()
    # เรียกฟังก์ชัน main
