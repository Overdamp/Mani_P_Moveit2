# 🚀 Mani-P Action System: Run Guide & Cheat Sheet (คู่มือการรันระบบและคำสั่งลัด)

This document provides a quick reference for building, launching, and testing the Action-Based Scheduler system.
(เอกสารนี้รวบรวมข้อมูลอ้างอิงด่วนสำหรับการ Build, Launch และทดสอบระบบ Action-Based Scheduler)

## 1. Build & Setup (การติดตั้งและคอมไพล์)
**Crucial Step:** You must rebuild the workspace whenever you change Action Definitions (`.action` files) or URDF/SRDF.
(ขั้นตอนสำคัญ: คุณต้อง Rebuild Workspace ทุกครั้งที่มีการแก้ไขไฟล์ Action Definition (`.action`) หรือไฟล์ URDF/SRDF)

```bash
cd ~/Ros2_Directory/Mani_P_Moveit2_ws

# Build specific packages (faster) (เลือก Build เฉพาะ Package ที่จำเป็น เพื่อความรวดเร็ว)
colcon build --packages-select mani_p_actions mani_p_task_scheduler mani_p_description --symlink-install

# Source the environment (โหลด Environment)
source install/setup.bash
```

---

## 2. Launch System (เริ่มระบบ)
Start all Action Servers (Approach, Level, MoveLinear, Gripper) in a single terminal.
(เริ่มการทำงานของ Action Server ทั้งหมด (เดินเข้าหา, ปรับระนาบ, เดินเส้นตรง, มือจับ) ใน Terminal เดียว)

```bash
ros2 launch mani_p_task_scheduler action_servers.launch.py
```
*Wait until you see "Ready" messages for all servers.*
(รอจนกว่าจะเห็นข้อความ "Ready" ขึ้นครบทุก Server)

---

## 3. Run Scheduler (Mission Control) (เริ่มภารกิจ)
Execute the full sequence (Approach -> Level -> etc.) via the Scheduler node.
(สั่งการทำงานตามลำดับขั้นตอน (เข้าหา -> ปรับระนาบ -> ฯลฯ) ผ่าน Scheduler Node)

```bash
# Usage: ros2 run mani_p_task_scheduler action_scheduler.py [tag_name]
ros2 run mani_p_task_scheduler action_scheduler.py tag2
```

---

## 4. Manual Testing (CLI) (การทดสอบด้วยมือผ่าน Command Line)
You can test each action individually using `ros2 action send_goal`.
(คุณสามารถทดสอบแต่ละ Action แยกกันได้โดยใช้คำสั่ง `ros2 action send_goal`)

### 🏃 Approach Tag (เดินเข้าหา Tag)
Move to a specific distance from an AprilTag.
(เคลื่อนที่ไปยังระยะที่กำหนดจาก AprilTag)
```bash
ros2 action send_goal /approach_tag mani_p_actions/action/ApproachTag "{tag_name: 'tag2', distance: 0.25, roll_offset: 0.0}"
```

### 📐 Adjust Level (ปรับระนาบมือจับ)
Align gripper to be level (parallel to ground/camera).
(ปรับหมุนมือจับให้ขนานกับพื้นหรือกล้อง)
```bash
ros2 action send_goal /adjust_level mani_p_actions/action/AdjustLevel "{target_roll: 0.0}"
```

### 📏 Move Linear (Cartesian) (เคลื่อนที่แนวเส้นตรง)
Move forward (+) or backward (-) in meters.
(เคลื่อนที่ไปข้างหน้า (+) หรือถอยหลัง (-) เป็นหน่วยเมตร)
```bash
# Move Forward 10cm (เดินหน้า 10 ซม.)
ros2 action send_goal /move_linear mani_p_actions/action/MoveLinear "{distance: 0.1}"

# Move Backward 5cm (ถอยหลัง 5 ซม.)
ros2 action send_goal /move_linear mani_p_actions/action/MoveLinear "{distance: -0.05}"
```

### 🖐️ Gripper Control (ควบคุมมือจับ)
Open or Close the gripper.
(สั่งกางหรือหุบมือจับ)
```bash
# Open (กางออก)
ros2 action send_goal /gripper_control mani_p_actions/action/GripperControl "{open: true}"

# Close (Tight Grip) (หุบเข้า - หนีบแน่น)
ros2 action send_goal /gripper_control mani_p_actions/action/GripperControl "{open: false}"
```

---

## 5. Troubleshooting (การแก้ปัญหาเบื้องต้น)
*   **"Action Server not available"**: Make sure `action_servers.launch.py` is running.
    (ตรวจสอบว่ารัน `action_servers.launch.py` หรือยัง)
*   **"Planning Failed"**: Target might be out of reach or in collision. Try moving the robot to a "Home" position first.
    (เป้าหมายอาจจะไกลเกินไปหรือติดชน ให้ลองขยับหุ่นไปท่า Home ก่อน)
*   **Gripper not closing tight enough**: The URDF limit has been extended. Ensure you have rebuilt `mani_p_description`.
    (ถ้ามือจับหนีบไม่แน่น: เราได้ขยาย Limit ใน URDF แล้ว ให้แน่ใจว่าได้ Rebuild `mani_p_description` แล้ว)
