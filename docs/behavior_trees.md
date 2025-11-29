# Behavior Tree Libraries for ROS 2

If you want to move beyond the simple custom implementation in `BT_scheduler.py`, here are the industry-standard libraries:

## 1. py_trees (Python) 🐍
This is the most popular BT library for Python in ROS. It is feature-rich, supports visualization, and has a ROS 2 extension (`py_trees_ros`).

### Key Concepts
*   **Composites**: `Sequence`, `Selector`, `Parallel`
*   **Decorators**: `Inverter`, `OneShot`, `Timeout`
*   **Leaves**: `Action`, `Condition`
*   **Blackboard**: Shared memory for passing data between nodes.

### Example Usage
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

### Visualization
You can visualize the tree structure using `py_trees_ros_viewer` or by exporting to dot/png.

---

## 2. BehaviorTree.CPP (C++) ⚙️
This is the standard for high-performance C++ robots (used by Nav2). It uses an XML-based tree definition, allowing you to change behavior without recompiling code.

### Key Features
*   **XML Loading**: Define logic in XML, implement actions in C++.
*   **Groot**: A GUI tool to edit and monitor trees in real-time.
*   **Reactive**: Highly optimized for complex, reactive behaviors.

### Example XML
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

## Recommendation
*   **Stick with Python (`py_trees`)** for now if you want rapid prototyping and ease of use.
*   **Switch to C++ (`BehaviorTree.CPP`)** if you need strict real-time performance or want to use the **Groot** visualizer.
