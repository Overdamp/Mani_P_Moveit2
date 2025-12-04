# 🚀 Mani-P Action System: Run Guide & Cheat Sheet

This document provides a quick reference for building, launching, and testing the Action-Based Scheduler system.

## 1. Build & Setup
**Crucial Step:** You must rebuild the workspace whenever you change Action Definitions (`.action` files) or URDF/SRDF.

```bash
cd ~/Ros2_Directory/Mani_P_Moveit2_ws

# Build specific packages (faster)
colcon build --packages-select mani_p_actions mani_p_task_scheduler mani_p_description --symlink-install

# Source the environment
source install/setup.bash
```

---

## 2. Launch System
Start all Action Servers (Approach, Level, MoveLinear, Gripper) in a single terminal.

```bash
ros2 launch mani_p_task_scheduler action_servers.launch.py
```
*Wait until you see "Ready" messages for all servers.*

---

## 3. Run Scheduler (Mission Control)
Execute the full sequence (Approach -> Level -> etc.) via the Scheduler node.

```bash
# Usage: ros2 run mani_p_task_scheduler action_scheduler.py [tag_name]
ros2 run mani_p_task_scheduler action_scheduler.py tag2
```

---

## 4. Manual Testing (CLI)
You can test each action individually using `ros2 action send_goal`.

### 🏃 Approach Tag
Move to a specific distance from an AprilTag.
```bash
ros2 action send_goal /approach_tag mani_p_actions/action/ApproachTag "{tag_name: 'tag2', distance: 0.25, roll_offset: 0.0}"
```

### 📐 Adjust Level
Align gripper to be level (parallel to ground/camera).
```bash
ros2 action send_goal /adjust_level mani_p_actions/action/AdjustLevel "{target_roll: 0.0}"
```

### 📏 Move Linear (Cartesian)
Move forward (+) or backward (-) in meters.
```bash
# Move Forward 10cm
ros2 action send_goal /move_linear mani_p_actions/action/MoveLinear "{distance: 0.1}"

# Move Backward 5cm
ros2 action send_goal /move_linear mani_p_actions/action/MoveLinear "{distance: -0.05}"
```

### 🖐️ Gripper Control
Open or Close the gripper.
```bash
# Open
ros2 action send_goal /gripper_control mani_p_actions/action/GripperControl "{open: true}"

# Close (Tight Grip)
ros2 action send_goal /gripper_control mani_p_actions/action/GripperControl "{open: false}"
```

---

## 5. Troubleshooting
*   **"Action Server not available"**: Make sure `action_servers.launch.py` is running.
*   **"Planning Failed"**: Target might be out of reach or in collision. Try moving the robot to a "Home" position first.
*   **Gripper not closing tight enough**: The URDF limit has been extended. Ensure you have rebuilt `mani_p_description`.
