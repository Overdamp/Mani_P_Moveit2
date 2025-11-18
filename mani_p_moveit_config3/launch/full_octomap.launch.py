# ลบ 'import os' ออก เพราะเราจะไม่ใช้มัน
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution # <-- นำเข้าตัวต่อ Path ของ ROS 2

def generate_launch_description():

    # 1. ค้นหาไฟล์ config ที่เราเพิ่งสร้าง (วิธีที่ถูกต้อง)
    #    เราต้องใช้ PathJoinSubstitution ของ ROS 2 แทน os.path.join
    config_file = PathJoinSubstitution([
        FindPackageShare('mani_p_moveit_config3'), # (เปลี่ยนชื่อ package ถ้าคุณเก็บ .yaml ไว้ที่อื่น)
        'config',
        'octomap_config.yaml'
    ])

    # 2. สร้าง map frame (เหมือนเดิม)
    world_to_map_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_map',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'map']
    )

    # 3. เชื่อม TF ของ RealSense (เหมือนเดิม)
    realsense_tf_stitch_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='realsense_tf_stitch',
        output='screen',
        arguments=['0', '0', '0',     # X, Y, Z
                   '0', '0', '0',     # R, P, Y
                   'd435i_camera_link', # Parent Frame (จาก URDF)
                   'camera_link']        # Child Frame (จาก Node กล้อง RealSense)
    )

    # 4. Octomap Server (เหมือนเดิม, แต่ตอนนี้ config_file ถูกต้องแล้ว)
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server', # <-- ชื่อนี้ต้องตรงกับใน .yaml
        output='screen',
        parameters=[config_file] # <-- ส่ง object ที่ถูกต้องเข้าไป
    )

    return LaunchDescription([
        world_to_map_node,
        realsense_tf_stitch_node,
        octomap_server_node
    ])