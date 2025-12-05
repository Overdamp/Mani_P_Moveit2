# 📏 Robot Environment Calibration Guide (คู่มือการปรับเทียบสภาพแวดล้อมหุ่นยนต์)

This guide explains how to use the `calibration_tool.py` to verify and tune the alignment between the real world and the robot's digital twin (URDF/TF).
(คู่มือนี้อธิบายวิธีการใช้ `calibration_tool.py` เพื่อตรวจสอบและปรับจูนความตรงกันระหว่างโลกจริงกับแบบจำลองดิจิทัลของหุ่นยนต์ (URDF/TF))

## 🎯 Objective (วัตถุประสงค์)
Ensure that the distance between the **Camera**, **Shelf**, and **Target Object** in the simulation matches reality. This is critical for accurate picking.
(เพื่อให้มั่นใจว่าระยะห่างระหว่าง **กล้อง**, **ชั้นวาง**, และ **วัตถุเป้าหมาย** ใน Simulation ตรงกับความเป็นจริง ซึ่งสำคัญมากต่อความแม่นยำในการหยิบจับ)

## 🛠️ Prerequisites (สิ่งที่ต้องเตรียม)
1.  **ZED Camera** is running and publishing TF.
    (กล้อง ZED ต้องทำงานและส่งค่า TF อยู่)
2.  **AprilTag Detection** is running (`apriltag_ros` or similar).
    (ระบบตรวจจับ AprilTag ต้องทำงานอยู่)
3.  **Tags are visible**:
    (ต้องมองเห็น Tag ดังนี้):
    *   **Shelf Tag** (Reference): Fixed on the shelf (e.g., ID 0).
        (Tag ชั้นวาง (อ้างอิง): ติดอยู่กับชั้นวาง เช่น ID 0)
    *   **Cube Tag** (Target): Placed on the object (e.g., ID 2).
        (Tag ลูกบาศก์ (เป้าหมาย): ติดอยู่บนวัตถุ เช่น ID 2)

## 🚀 How to Run the Tool (วิธีรันเครื่องมือ)

1.  **Build (if new):** (คอมไพล์ ถ้ายังไม่เคยทำ)
    ```bash
    colcon build --packages-select mani_p_test --symlink-install
    source install/setup.bash
    ```

2.  **Run:** (รันโปรแกรม)
    ```bash
    # Usage: ros2 run mani_p_test calibration_tool.py [camera_frame] [shelf_tag] [cube_tag]
    
    # Example (Default frames):
    ros2 run mani_p_test calibration_tool.py
    
    # Example (Custom frames):
    ros2 run mani_p_test calibration_tool.py zed_left_camera_optical_frame tag36h11:0 tag36h11:2
    ```

## 📊 How to Interpret Output (วิธีอ่านค่า)

The tool prints the XYZ coordinates and straight-line Distance every second.
(เครื่องมือจะแสดงพิกัด XYZ และระยะทางตรง (Distance) ทุกๆ วินาที)

```text
[Camera -> Shelf Tag]
   XYZ: (0.12, 0.05, 0.85)
   Distance: 0.8602 m

[Camera -> Cube Tag]
   XYZ: (-0.05, 0.02, 0.60)
   Distance: 0.6024 m

[Shelf Tag -> Cube Tag]
   XYZ: (-0.17, -0.03, -0.25)
   Distance: 0.3036 m
```

## 🔧 Calibration Steps (ขั้นตอนการปรับเทียบ)

### Step 1: Verify Camera Position (ตรวจสอบตำแหน่งกล้อง)
1.  Measure the **real distance** from the Camera lens to the Shelf Tag using a tape measure.
    (วัด **ระยะจริง** จากเลนส์กล้องไปยัง Tag ชั้นวางด้วยตลับเมตร)
2.  Compare with `[Camera -> Shelf Tag] Distance`.
    (เปรียบเทียบกับค่า `Distance` ที่โปรแกรมแสดง)
3.  **If different:** Adjust the Static TF of the camera in your launch file (usually `static_transform_publisher` args).
    (**ถ้าไม่ตรง:** ให้ปรับค่า Static TF ของกล้องใน Launch file)

### Step 2: Verify Shelf Position (URDF) (ตรวจสอบตำแหน่งชั้นวาง)
1.  If the Shelf Tag is fixed to the shelf, its position relative to the robot base should be constant.
    (ถ้า Tag ติดตายตัวกับชั้นวาง ตำแหน่งเทียบกับฐานหุ่นยนต์ต้องคงที่)
2.  Check if the `Shelf Tag -> Robot Base` TF matches your URDF model.
    (เช็คว่าค่า TF จาก Tag ไปยังฐานหุ่น ตรงกับโมเดล URDF หรือไม่)
3.  **If different:** Adjust the Shelf link position in your URDF or the scene description.
    (**ถ้าไม่ตรง:** ให้ปรับตำแหน่ง Link ชั้นวางใน URDF)

### Step 3: Verify Object Detection (ตรวจสอบการตรวจจับวัตถุ)
1.  Place the Cube at a known distance from the Shelf Tag (e.g., 30cm).
    (วางลูกบาศก์ที่ระยะที่รู้แน่นอนจาก Tag ชั้นวาง เช่น 30 ซม.)
2.  Check `[Shelf Tag -> Cube Tag] Distance`.
    (เช็คค่าระยะ `Shelf Tag -> Cube Tag`)
3.  It should be close to 0.30m.
    (ค่าควรจะใกล้เคียง 0.30m)
4.  **If different:** Check camera calibration (intrinsics) or tag size settings in `apriltag_ros`.
    (**ถ้าไม่ตรง:** ให้เช็คการ Calibrate เลนส์กล้อง หรือการตั้งค่าขนาด Tag)

## 🛠️ Tuning Strategies (The "Easiest" Fixes) (กลยุทธ์การปรับจูนแบบง่าย)

If you find a discrepancy, here are the easiest ways to fix it, ranked by simplicity:
(ถ้าเจอความคลาดเคลื่อน นี่คือวิธีแก้ที่ง่ายที่สุด เรียงตามลำดับ)

### 1. Adjust Static TF (Visual Alignment) 🥇 **Easiest** (ปรับ Static TF - ง่ายสุด)
If the entire visual world seems shifted (e.g., robot grabs 2cm to the right), shift the camera frame.
(ถ้าภาพรวมในโลกเสมือนดูเบี้ยวๆ เช่น หุ่นหยิบขวาไป 2 ซม. ให้ขยับเฟรมกล้องหลอกๆ เอาเลย)
*   **Command:** Run this in a separate terminal to override/test:
    (คำสั่งสำหรับทดสอบปรับค่า):
    ```bash
    # Args: x y z yaw pitch roll parent_frame child_frame
    ros2 run tf2_ros static_transform_publisher 0.02 0 0 0 0 0 Base_link zed_camera_link
    ```
*   **How to tune:** Adjust the X/Y/Z values until the TF distance matches the real distance.
    (วิธีจูน: ปรับค่า X/Y/Z จนกว่าระยะในโปรแกรมจะตรงกับระยะจริง)
*   **Permanent Fix:** Add this node to your `mani_perception.launch.py`.
    (วิธีแก้ถาวร: เอาคำสั่งนี้ไปใส่ใน `mani_perception.launch.py`)

### 2. Check Tag Size (Scale Error) 🥈 (เช็คขนาด Tag)
If objects look closer/further than they are (Z-axis error), check the tag size configuration.
(ถ้าของดูใกล้หรือไกลกว่าความเป็นจริง (แกน Z เพี้ยน) ให้เช็คการตั้งค่าขนาด Tag)
*   **File:** `mani_p_moveit_config3/config/tags.yaml` (or similar)
*   **Check:** Ensure `size` matches the printed tag edge length (meters).
    (เช็คว่าค่า `size` ตรงกับความยาวขอบ Tag จริงๆ ในหน่วยเมตร)
    *   Example: 5cm = `0.05`

### 3. Adjust TCP Offset (Picking Error) 🥉 (ปรับระยะ TCP)
If the vision is correct but the gripper misses (e.g., stops too early), adjust the "Tool Center Point".
(ถ้าค่า Vision ตรงแล้ว แต่หุ่นยังหยิบพลาด (เช่น หยุดก่อนถึงของ) ให้ปรับจุด TCP)
*   **File:** `mani_p_description/robot/mani_p.core2.xacro`
*   **Edit:** Change the `tcp_joint` origin Z value.
    (แก้ไขค่า Z ของ `tcp_joint`)
    ```xml
    <joint name="tcp_joint" type="fixed">
      <origin xyz="0 0 0.25" ... /> <!-- Increase to reach further (เพิ่มค่าเพื่อให้ยื่นออกไปอีก) -->
    ```

## 💡 Tips (เกร็ดความรู้)
*   **X, Y, Z axes:** Remember that in optical frames, **Z is forward**, **X is right**, **Y is down**.
    (จำไว้ว่าในเฟรมกล้อง (Optical Frame): **Z คือด้านหน้า**, **X คือด้านขวา**, **Y คือด้านล่าง**)
