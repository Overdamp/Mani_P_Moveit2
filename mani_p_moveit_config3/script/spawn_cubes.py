#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
import math
from collections import deque, defaultdict

class CubeSpawner(Node):

    def __init__(self):
        super().__init__('cube_spawner')

        # ==========================================
        # 🛠️ CONFIG ZONE 🛠️
        # ==========================================
        self.target_tags = ["tag1", "tag2", "tag3"]
        self.base_frame = "Base_link"
        self.cube_size = 0.05 
        
        # Smoothing Config
        self.buffer_size = 5  # จำนวน Frame ที่นำมาเฉลี่ย
        self.timer_period = 0.1 # 10Hz
        # ==========================================

        self.collision_pub = self.create_publisher(CollisionObject, '/collision_object', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Data Storage for Smoothing
        # Key: tag_id, Value: dict of deques
        self.buffers = defaultdict(lambda: {
            'x': deque(maxlen=self.buffer_size),
            'y': deque(maxlen=self.buffer_size),
            'z': deque(maxlen=self.buffer_size),
            'sin_yaw': deque(maxlen=self.buffer_size),
            'cos_yaw': deque(maxlen=self.buffer_size)
        })

        self.timer = self.create_timer(self.timer_period, self.update_objects)
        self.get_logger().info(f'Cube Spawner (Smoothed) Started for: {self.target_tags}')

    def get_euler_from_quaternion(self, q):
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (q.w * q.y - q.z * q.x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi/2, sinp)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

    def update_objects(self):
        for tag_id in self.target_tags:
            try:
                # 1. Get Raw Transform
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame, tag_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05))

                raw_x = transform.transform.translation.x
                raw_y = transform.transform.translation.y
                raw_z = transform.transform.translation.z
                _, _, raw_yaw = self.get_euler_from_quaternion(transform.transform.rotation)

                # 2. Add to Buffer
                buf = self.buffers[tag_id]
                buf['x'].append(raw_x)
                buf['y'].append(raw_y)
                buf['z'].append(raw_z)
                buf['sin_yaw'].append(math.sin(raw_yaw))
                buf['cos_yaw'].append(math.cos(raw_yaw))

                if len(buf['x']) < 1: continue

                # 3. Calculate Average
                avg_x = sum(buf['x']) / len(buf['x'])
                avg_y = sum(buf['y']) / len(buf['y'])
                avg_z = sum(buf['z']) / len(buf['z'])
                
                avg_sin = sum(buf['sin_yaw']) / len(buf['sin_yaw'])
                avg_cos = sum(buf['cos_yaw']) / len(buf['cos_yaw'])
                avg_yaw = math.atan2(avg_sin, avg_cos)

                # 4. Construct Object (Lock Roll/Pitch = 0)
                obj = CollisionObject()
                obj.header.frame_id = self.base_frame
                obj.id = f"cube_{tag_id}" 

                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [self.cube_size, self.cube_size, self.cube_size]

                pose = PoseStamped()
                pose.pose.position.x = avg_x
                pose.pose.position.y = avg_y
                pose.pose.position.z = avg_z 
                
                # Force Roll=0, Pitch=0
                q_list = self.get_quaternion_from_euler(0.0, 0.0, avg_yaw)
                pose.pose.orientation = Quaternion(x=q_list[0], y=q_list[1], z=q_list[2], w=q_list[3])

                obj.primitives.append(primitive)
                obj.primitive_poses.append(pose.pose)
                obj.operation = CollisionObject.ADD

                self.collision_pub.publish(obj)

            except Exception:
                continue

def main(args=None):
    rclpy.init(args=args)
    node = CubeSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()