# Mani-P Robot Project Overview (ภาพรวมโครงการหุ่นยนต์ Mani-P)

## 📖 Introduction (บทนำ)
This project aims to develop a robust **Pick-and-Place** system for the **Mani-P** (6-DOF Manipulator) using ROS 2 Humble and MoveIt 2. The system features a **Dual-Camera Setup** (Global ZED 2i + Eye-in-Hand RealSense D435i) to achieve high-precision grasping of objects from a shelf.
(โครงการนี้มีจุดมุ่งหมายเพื่อพัฒนาระบบ **Pick-and-Place** ที่แข็งแกร่งสำหรับ **Mani-P** (แขนกล 6 แกน) โดยใช้ ROS 2 Humble และ MoveIt 2 ระบบนี้ใช้ **การติดตั้งกล้องคู่** (กล้องหลัก ZED 2i + กล้องติดปลายมือ RealSense D435i) เพื่อให้สามารถหยิบจับวัตถุจากชั้นวางได้อย่างแม่นยำสูง)

## 🏗️ System Architecture (สถาปัตยกรรมระบบ)

### Hardware (ฮาร์ดแวร์)
*   **Manipulator**: Custom 6-DOF arm powered by Dynamixel motors.
    (แขนกล: แขนกล 6 แกนแบบสร้างเอง ขับเคลื่อนด้วยมอเตอร์ Dynamixel)
*   **Controller**: NVIDIA Jetson Orin Nano.
    (ตัวควบคุม: NVIDIA Jetson Orin Nano)
*   **Sensors**:
    (เซนเซอร์):
    *   **Stereolabs ZED 2i**: Fixed global camera for environment perception.
        (กล้อง ZED 2i: ติดตั้งตายตัวสำหรับมองภาพรวมสภาพแวดล้อม)
    *   **Intel RealSense D435i**: Mounted on the gripper for fine visual servoing.
        (กล้อง RealSense D435i: ติดตั้งที่มือจับสำหรับการเล็งตำแหน่งละเอียด)

### Software Stack (ซอฟต์แวร์)
*   **OS**: Ubuntu 22.04 (Jammy Jellyfish)
*   **Middleware**: ROS 2 Humble
*   **Motion Planning**: MoveIt 2
*   **Vision**: AprilTag detection, Point Cloud processing.

---

## 📦 Package Summary (สรุป Package)

The workspace contains several packages, each serving a specific role:
(Workspace ประกอบด้วยหลาย Package แต่ละอันมีหน้าที่เฉพาะ:)

### 1. Core Packages (Custom) (Package หลัก - สร้างเอง)

| Package Name | Description (คำอธิบาย) |
| :--- | :--- |
| **`mani_p_moveit_config3`** | **[MAIN]** Contains MoveIt 2 configuration, launch files, and high-level control scripts (`approach_tag`, `visual_servo`, `cartesian_push`). <br> (**[หลัก]** เก็บค่า Config ของ MoveIt 2, ไฟล์ Launch และสคริปต์ควบคุมระดับสูง) |
| **`mani_p_description`** | Contains the robot's URDF/Xacro models, meshes, and sensor definitions. <br> (เก็บโมเดล URDF/Xacro ของหุ่นยนต์, ไฟล์ Mesh และการนิยามเซนเซอร์) |
| **`dynamixel_hardware`** | Custom `ros2_control` hardware interface plugin for communicating with Dynamixel motors. <br> (ปลั๊กอิน `ros2_control` สำหรับสื่อสารกับมอเตอร์ Dynamixel) |

### 2. Driver & Dependency Packages (ไดรเวอร์และไลบรารีที่ต้องใช้)

| Package Name | Description (คำอธิบาย) |
| :--- | :--- |
| **`realsense-ros2`** | Official ROS 2 driver for Intel RealSense cameras. |
| **`zed-ros2-wrapper`** | Official ROS 2 wrapper for Stereolabs ZED cameras. |
| **`DynamixelSDK`** | Low-level C++ library for Dynamixel communication protocol. |
| **`dynamixel-workbench`** | Tools and examples for Dynamixel motors. |
| **`robotis_manipulator`** | Base library for Robotis manipulator kinematics. |

---

## 🚀 Key Workflows (ขั้นตอนการทำงานหลัก)

### Pick & Place Strategy (Coarse-to-Fine) (กลยุทธ์การหยิบวาง - หยาบไปละเอียด)
1.  **Global Detection**: ZED camera detects the shelf and target object tags.
    (ตรวจจับภาพรวม: กล้อง ZED ตรวจจับชั้นวางและ Tag ของวัตถุเป้าหมาย)
2.  **Coarse Approach**: Robot moves to a standoff distance (~25cm) using `approach_tag_smart.py`.
    (เข้าหาแบบหยาบ: หุ่นยนต์เคลื่อนที่ไปที่ระยะเตรียมพร้อม (~25 ซม.) โดยใช้ `approach_tag_smart.py`)
3.  **Fine Alignment**: RealSense camera performs **Visual Servoing** (`visual_servo_align.py`) to center the gripper.
    (เล็งละเอียด: กล้อง RealSense ทำการ **Visual Servoing** (`visual_servo_align.py`) เพื่อจัดตำแหน่งมือจับให้อยู่กึ่งกลาง)
4.  **Grasp**: Robot performs a blind linear push (`cartesian_push.py`) to grasp the object.
    (หยิบ: หุ่นยนต์ดันตัวไปข้างหน้าเป็นเส้นตรง (`cartesian_push.py`) เพื่อหยิบวัตถุ)

### Deployment Modes (โหมดการใช้งาน)
*   **Headless (Jetson)**: Runs drivers and motion planning backend.
    (รันไดรเวอร์และระบบวางแผนการเคลื่อนที่ (ไม่มีหน้าจอ))
    *   `ros2 launch mani_p_moveit_config3 headless_controller.launch.py`
*   **Visualization (Laptop)**: Runs RViz for monitoring and command.
    (รัน RViz สำหรับดูสถานะและสั่งงาน (บนแล็ปท็อป))
    *   `ros2 launch mani_p_moveit_config3 visualize_client.launch.py`

## 📂 Directory Structure (Key Files) (โครงสร้างไฟล์สำคัญ)

```text
src/
├── mani_p_moveit_config3/
│   ├── launch/
│   │   ├── headless_controller.launch.py  # For Jetson (สำหรับ Jetson)
│   │   ├── visualize_client.launch.py     # For Laptop (สำหรับ Laptop)
│   │   └── mani_perception.launch.py      # Vision Pipeline (ระบบการมองเห็น)
│   ├── script/
│   │   ├── approach_tag_smart.py          # Coarse Approach (เข้าหาแบบหยาบ)
│   │   ├── visual_servo_align.py          # Fine Alignment (เล็งละเอียด)
│   │   └── cartesian_push.py              # Linear Push (ดันเส้นตรง)
│   └── config/                            # MoveIt & Controller Configs
├── mani_p_description/
│   ├── robot/
│   │   ├── mani_p.urdf2.xacro             # Top-level URDF (ไฟล์ URDF หลัก)
│   │   ├── mani_p.core2.xacro             # Arm Links/Joints (ลิ้งค์และข้อต่อแขน)
│   │   └── mani_p.sensors.xacro           # Camera Mounts (จุดติดกล้อง)
└── docs/                                  # This documentation (เอกสารชุดนี้)
```
