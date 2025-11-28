# Mani-P Robot Project Overview

## 📖 Introduction
This project aims to develop a robust **Pick-and-Place** system for the **Mani-P** (6-DOF Manipulator) using ROS 2 Humble and MoveIt 2. The system features a **Dual-Camera Setup** (Global ZED 2i + Eye-in-Hand RealSense D435i) to achieve high-precision grasping of objects from a shelf.

## 🏗️ System Architecture

### Hardware
*   **Manipulator**: Custom 6-DOF arm powered by Dynamixel motors.
*   **Controller**: NVIDIA Jetson Orin Nano.
*   **Sensors**:
    *   **Stereolabs ZED 2i**: Fixed global camera for environment perception.
    *   **Intel RealSense D435i**: Mounted on the gripper for fine visual servoing.

### Software Stack
*   **OS**: Ubuntu 22.04 (Jammy Jellyfish)
*   **Middleware**: ROS 2 Humble
*   **Motion Planning**: MoveIt 2
*   **Vision**: AprilTag detection, Point Cloud processing.

---

## 📦 Package Summary

The workspace contains several packages, each serving a specific role:

### 1. Core Packages (Custom)

| Package Name | Description |
| :--- | :--- |
| **`mani_p_moveit_config3`** | **[MAIN]** Contains MoveIt 2 configuration, launch files, and high-level control scripts (`approach_tag`, `visual_servo`, `cartesian_push`). |
| **`mani_p_description`** | Contains the robot's URDF/Xacro models, meshes, and sensor definitions. |
| **`dynamixel_hardware`** | Custom `ros2_control` hardware interface plugin for communicating with Dynamixel motors. |

### 2. Driver & Dependency Packages

| Package Name | Description |
| :--- | :--- |
| **`realsense-ros2`** | Official ROS 2 driver for Intel RealSense cameras. |
| **`zed-ros2-wrapper`** | Official ROS 2 wrapper for Stereolabs ZED cameras. |
| **`DynamixelSDK`** | Low-level C++ library for Dynamixel communication protocol. |
| **`dynamixel-workbench`** | Tools and examples for Dynamixel motors. |
| **`robotis_manipulator`** | Base library for Robotis manipulator kinematics. |

---

## 🚀 Key Workflows

### Pick & Place Strategy (Coarse-to-Fine)
1.  **Global Detection**: ZED camera detects the shelf and target object tags.
2.  **Coarse Approach**: Robot moves to a standoff distance (~25cm) using `approach_tag_smart.py`.
3.  **Fine Alignment**: RealSense camera performs **Visual Servoing** (`visual_servo_align.py`) to center the gripper.
4.  **Grasp**: Robot performs a blind linear push (`cartesian_push.py`) to grasp the object.

### Deployment Modes
*   **Headless (Jetson)**: Runs drivers and motion planning backend.
    *   `ros2 launch mani_p_moveit_config3 headless_controller.launch.py`
*   **Visualization (Laptop)**: Runs RViz for monitoring and command.
    *   `ros2 launch mani_p_moveit_config3 visualize_client.launch.py`

## 📂 Directory Structure (Key Files)

```text
src/
├── mani_p_moveit_config3/
│   ├── launch/
│   │   ├── headless_controller.launch.py  # For Jetson
│   │   ├── visualize_client.launch.py     # For Laptop
│   │   └── mani_perception.launch.py      # Vision Pipeline
│   ├── script/
│   │   ├── approach_tag_smart.py          # Coarse Approach
│   │   ├── visual_servo_align.py          # Fine Alignment
│   │   └── cartesian_push.py              # Linear Push
│   └── config/                            # MoveIt & Controller Configs
├── mani_p_description/
│   ├── robot/
│   │   ├── mani_p.urdf2.xacro             # Top-level URDF
│   │   ├── mani_p.core2.xacro             # Arm Links/Joints
│   │   └── mani_p.sensors.xacro           # Camera Mounts
└── docs/                                  # This documentation
```
