# Mani-P MoveIt2 Configuration (Pick & Place)

This package (`mani_p_moveit_config3`) contains the MoveIt2 configuration and control scripts for the **Mani-P Manipulator**. It is designed to perform robust pick-and-place operations using a **Dual-Camera Setup** (Global + Eye-in-Hand).

## 🤖 Hardware Setup

*   **Robot Arm**: Mani-P (6-DOF Custom Manipulator)
*   **Controller**: Jetson Orin Nano (Running ROS 2 Humble)
*   **Sensors**:
    1.  **Global Camera**: Stereolabs ZED 2i (Fixed Mount) - Used for object detection and coarse approach.
    2.  **Local Camera**: Intel RealSense D435i (Eye-in-Hand) - Used for fine alignment (Visual Servoing).
*   **Gripper**: Custom 2-Finger Gripper.

## 🧠 System Architecture

The system operates on a **Coarse-to-Fine** strategy:

1.  **Perception**:
    *   **ZED 2i**: Detects AprilTags on the shelf and objects. Publishes TF frames.
    *   **RealSense D435i**: Provides close-range high-precision TF frames for the target tag.
2.  **Planning**:
    *   **MoveIt 2**: Handles kinematics, collision checking, and path planning.
    *   **Custom Scripts**: Orchestrate the pick-and-place sequence.
3.  **Control**:
    *   **ros2_control**: Low-level hardware interface for Dynamixel servos.

## 🚀 Deployment (Headless vs GUI)

This project is designed to run in a distributed manner:

### 1. On the Robot (Jetson Orin Nano) - Headless
Run the core drivers and motion planning backend without a GUI to save resources.

```bash
ros2 launch mani_p_moveit_config3 headless_controller.launch.py
```
*Starts: `ros2_control`, `move_group`, `robot_state_publisher`.*

### 2. On the Laptop (Operator) - Visualization
Connect to the robot to visualize the state and send high-level commands.

```bash
ros2 launch mani_p_moveit_config3 visualize_client.launch.py
```
*Starts: `RViz2` with MoveIt plugin.*

> **Note**: Ensure both machines are on the same network and `ROS_DOMAIN_ID` is set correctly.

---

## 🛠️ Operational Workflow (Pick & Place)

The pick-and-place task is broken down into modular Python scripts:

### Step 1: Perception & Environment
Start the perception pipeline to detect tags and spawn the shelf/objects in MoveIt.
```bash
ros2 launch mani_p_moveit_config3 mani_perception.launch.py
```

### Step 2: Coarse Approach (ZED)
Move the robot to a "Look Pose" or a standoff distance from the target using the Global Camera.
```bash
# Usage: ros2 run <pkg> approach_tag_smart.py <TAG_NAME> <DISTANCE>
ros2 run mani_p_moveit_config3 approach_tag_smart.py tag2 0.25
```
*   **Result**: Robot moves to 25cm in front of `tag2`.

### Step 3: Fine Alignment (RealSense) - **Visual Servoing**
Use the Eye-in-Hand camera to center the gripper perfectly with the tag.
```bash
# Usage: ros2 run <pkg> visual_servo_align.py <TAG_NAME>
ros2 run mani_p_moveit_config3 visual_servo_align.py tag2
```
*   **Result**: Robot adjusts X/Y position until error < 2mm.

### Step 4: Grasp (Cartesian Push)
Perform a blind linear push to insert the gripper and grasp the object.
```bash
# Usage: ros2 run <pkg> cartesian_push.py <DISTANCE>
ros2 run mani_p_moveit_config3 cartesian_push.py 0.15
```
*   **Result**: Robot moves forward 15cm in a straight line.

---

## 📂 Key Scripts

| Script | Description |
| :--- | :--- |
| `approach_tag_smart.py` | Uses MoveIt Constraints to approach a target tag while maintaining orientation. |
| `visual_servo_align.py` | Closed-loop control using RealSense TF to align the gripper with the target. |
| `cartesian_push.py` | Computes and executes a linear Cartesian path (Z-axis push). |
| `spawn_cubes.py` | Spawns collision objects (boxes) at detected tag locations. |
| `smart_soft_snap.py` | Spawns the shelf mesh aligned with gravity and filtered for stability. |

## 📦 Installation

1.  Clone the repository into your workspace `src`.
2.  Install dependencies:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```
3.  Build:
    ```bash
    colcon build --symlink-install
    ```
