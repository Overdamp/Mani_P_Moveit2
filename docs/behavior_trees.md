# Behavior Tree Libraries for ROS 2 (ไลบรารี Behavior Tree สำหรับ ROS 2)

If you want to move beyond the simple custom implementation in `BT_scheduler.py`, here are the industry-standard libraries:
(หากคุณต้องการขยับขยายจากระบบที่เขียนเองง่ายๆ ใน `BT_scheduler.py` นี่คือไลบรารีมาตรฐานที่นิยมใช้กันในอุตสาหกรรม:)

## 1. py_trees (Python) 🐍
This is the most popular BT library for Python in ROS. It is feature-rich, supports visualization, and has a ROS 2 extension (`py_trees_ros`).
(นี่คือไลบรารี BT ยอดนิยมที่สุดสำหรับ Python ใน ROS มีฟีเจอร์ครบครัน รองรับการแสดงผลภาพ (Visualization) และมีส่วนเสริมสำหรับ ROS 2 (`py_trees_ros`))

### Key Concepts (แนวคิดหลัก)
*   **Composites**: `Sequence`, `Selector`, `Parallel`
    (ตัวรวม: ทำงานตามลำดับ, เลือกทางเลือก, ทำงานขนานกัน)
*   **Decorators**: `Inverter`, `OneShot`, `Timeout`
    (ตัวตกแต่ง: กลับค่าผลลัพธ์, ทำครั้งเดียว, จับเวลา)
*   **Leaves**: `Action`, `Condition`
    (ใบ: การกระทำ, เงื่อนไข)
*   **Blackboard**: Shared memory for passing data between nodes.
    (กระดานดำ: หน่วยความจำร่วมสำหรับส่งข้อมูลระหว่างโหนด)

### Example Usage (ตัวอย่างการใช้งาน)
```python
import py_trees
import time

class ApproachAction(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachAction, self).__init__(name)

    def update(self):
        print(f"[{self.name}] Approaching...")
        # Return RUNNING, SUCCESS, or FAILURE
        return py_trees.common.Status.SUCCESS

# Build Tree
root = py_trees.composites.Sequence("MainSequence", memory=True)
approach = ApproachAction("ApproachTag")
grasp = py_trees.behaviours.Dummy("Grasp") # Built-in dummy for testing

root.add_children([approach, grasp])

# Run Tree
root.setup_with_descendants()
for i in range(5):
    root.tick_once()
    time.sleep(0.5)
```

### Visualization (การแสดงผลภาพ)
You can visualize the tree structure using `py_trees_ros_viewer` or by exporting to dot/png.
(คุณสามารถดูโครงสร้าง Tree ได้โดยใช้ `py_trees_ros_viewer` หรือส่งออกเป็นไฟล์ dot/png)

---

## 2. BehaviorTree.CPP (C++) ⚙️
This is the standard for high-performance C++ robots (used by Nav2). It uses an XML-based tree definition, allowing you to change behavior without recompiling code.
(นี่คือมาตรฐานสำหรับหุ่นยนต์ C++ ประสิทธิภาพสูง (ใช้ใน Nav2) ใช้การกำหนด Tree ด้วย XML ทำให้เปลี่ยนพฤติกรรมได้โดยไม่ต้องแก้โค้ดและคอมไพล์ใหม่)

### Key Features (ฟีเจอร์เด่น)
*   **XML Loading**: Define logic in XML, implement actions in C++.
    (โหลด XML: กำหนดตรรกะใน XML, เขียนการทำงานจริงใน C++)
*   **Groot**: A GUI tool to edit and monitor trees in real-time.
    (Groot: โปรแกรม GUI สำหรับแก้ไขและดูสถานะ Tree แบบเรียลไทม์)
*   **Reactive**: Highly optimized for complex, reactive behaviors.
    (ตอบสนองไว: ปรับแต่งมาอย่างดีสำหรับพฤติกรรมที่ซับซ้อนและต้องตอบสนองทันที)

### Example XML (ตัวอย่าง XML)
```xml
<root main_tree_to_execute = "MainTree">
    <BehaviorTree ID="MainTree">
        <Sequence name="PickAndPlace">
            <Action ID="Approach" target="tag2"/>
            <Action ID="Align"/>
            <Action ID="Grasp"/>
        </Sequence>
    </BehaviorTree>
</root>
```

## Recommendation (คำแนะนำ)
*   **Stick with Python (`py_trees`)** for now if you want rapid prototyping and ease of use.
    (ใช้ **Python (`py_trees`)** ไปก่อน หากต้องการความรวดเร็วในการทดลองและใช้งานง่าย)
*   **Switch to C++ (`BehaviorTree.CPP`)** if you need strict real-time performance or want to use the **Groot** visualizer.
    (เปลี่ยนไปใช้ **C++ (`BehaviorTree.CPP`)** หากต้องการประสิทธิภาพแบบ Real-time ขั้นสูง หรือต้องการใช้โปรแกรม **Groot**)
