import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():

    # --- 1. ค้นหา Path ของแพ็คเกจที่จำเป็น ---
    
    # Path ไปยัง MoveIt Config (!!! แก้ไข 'MY_MOVEIT_CONFIG_PKG' เป็นชื่อแพ็คเกจ MoveIt ของคุณ !!!)
    moveit_config_pkg_name = 'mani_p_moveit_config' # <--- !!! แก้ไขตรงนี้ !!!
    moveit_config_dir = get_package_share_directory(moveit_config_pkg_name)

    # Path ไปยัง URDF/Config (อ้างอิงจาก )
    description_pkg_name = 'mani_p_description'
    description_dir = get_package_share_directory(description_pkg_name)

    # --- 2. โหลด Robot Description (URDF) ---
    
    # ระบุไฟล์ URDF/Xacro ตัวหลัก
    # (เราจะสร้างไฟล์ .xacro ใหม่ที่ include ไฟล์ ros2_control แทน gazebo)
    # เราจะสร้างตัวแปรชี้ไปที่ไฟล์ xacro ใหม่
    urdf_xacro_file = os.path.join(description_dir, 'robot', 'mani_p.urdf.xacro')
    
    # **สำคัญมาก**: เราต้องบอก xacro ให้ include ไฟล์ .ros2_control.xacro ที่เราสร้าง
    # วิธีที่ง่ายที่สุดคือการแก้ไขไฟล์ 'open_manipulator_p_with_gripper_robot.urdf.xacro' 
    # ให้เปลี่ยนบรรทัด:
    # <xacro:include filename="$(find open_manipulator_p_description)/urdf/open_manipulator_p_with_gripper.gazebo.xacro" />
    # เป็น:
    # <xacro:include filename="$(find open_manipulator_p_description)/urdf/open_manipulator_p.ros2_control.xacro" />
    
    # โหลดและประมวลผล Xacro
    robot_description_doc = xacro.parse(open(urdf_xacro_file, 'r'))
    xacro.process_doc(robot_description_doc)
    robot_description_param = {'robot_description': robot_description_doc.toxml()}

    # --- 3. โหลด Controller Config (YAML) ---
    controllers_yaml_file = os.path.join(description_dir, 'config', 'ros2_controllers.yaml')

    # --- 4. สร้าง Node: Robot State Publisher ---
    # โหลด URDF และ publish TFs (/tf)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param]
    )

    # --- 5. สร้าง Node: Controller Manager (ros2_control_node) ---
    # นี่คือโหนดหลักของ ros2_control
    # มันจะโหลด Hardware Interface (จาก URDF) และจัดการ Controllers (จาก YAML)
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description_param, controllers_yaml_file],
        output='screen',
    )

    # --- 6. สร้าง Node "Spawners" ---
    # Spawners จะเริ่มทำงานหลังจาก Controller Manager (ข้อ 5) พร้อม
    
    # Spawner สำหรับ Joint State Broadcaster (จำเป็น)
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Spawner สำหรับ Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    # --- 7. สร้าง Node: MoveIt 2 (MoveGroup) ---
    # โหลดไฟล์ move_group.launch.py จากแพ็คเกจ MoveIt Config
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')
        )
    )

    # --- 8. สร้าง Node: RViz (สำหรับ MoveIt 2) ---
    # โหลดไฟล์ moveit_rviz.launch.py จากแพ็คเกจ MoveIt Config
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'moveit_rviz.launch.py')
        )
    )

    # --- 9. จัดลำดับการรัน (สำคัญ) ---
    # เราจะรัน Spawners (ข้อ 6) ก็ต่อเมื่อ Controller Manager (ข้อ 5) รันเสร็จแล้ว
    
    return LaunchDescription([
        # รัน RSP (ข้อ 4) และ CM (ข้อ 5) ทันที
        robot_state_publisher_node,
        controller_manager_node,

        # ใช้ RegisterEventHandler เพื่อรัน Spawners เมื่อ CM พร้อม
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=controller_manager_node,
                on_start=[
                    jsb_spawner,
                    arm_controller_spawner,
                ],
            )
        ),
        
        # รัน MoveIt (ข้อ 7) และ RViz (ข้อ 8)
        # เพิ่ม delay เล็กน้อยเพื่อให้แน่ใจว่า /joint_states พร้อมใช้งาน
        TimerAction(
            period=5.0, # รอ 5 วินาที
            actions=[
                move_group_launch,
                rviz_launch
            ]
        )
    ])