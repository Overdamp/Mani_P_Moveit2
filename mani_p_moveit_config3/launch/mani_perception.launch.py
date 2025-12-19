from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1. ZED Camera (Start Immediately)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('zed_wrapper'),
                    'launch',
                    'zed_camera.launch.py'
                ])
            ),
            launch_arguments={
                'camera_model': 'zed2i',
                'camera_name': 'zed_mani',
                'node_name': 'zed_node'
            }.items()
        ),

        # 1.5 RealSense D435i (Start Immediately)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         PathJoinSubstitution([
        #             FindPackageShare('realsense2_camera'),
        #             'launch',
        #             'rs_launch.py'
        #         ])
        #     ),
        #     launch_arguments={
        #         'camera_name': 'd435i_camera', # Match URDF name
        #         'device_type': 'd435i',
        #         'align_depth.enable': 'true'
        #     }.items()
        # ),

        # 2. AprilTag Detection (Wait 5s for Camera)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('mani_p_moveit_config3'),
                            'launch',
                            'apriltag_zed.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'camera_name': 'zed_mani'
                    }.items()
                )
            ]
        ),

        # 3. Smart Soft Snap (Wait 8s for Detections)
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='mani_p_moveit_config3',
                    executable='smart_soft_snap.py',
                    name='smart_soft_snap',
                    output='screen'
                )
            ]
        ),

        # 4. Spawn Cubes (Wait 10s)
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='mani_p_moveit_config3',
                    executable='spawn_cubes.py',
                    name='spawn_cubes',
                    output='screen'
                )
            ]
        ),

        # 5. Tag Leveler (Start with AprilTag)
        TimerAction(
            period=6.0, # Start shortly after AprilTag
            actions=[
                Node(
                    package='mani_p_moveit_config3',
                    executable='tag_leveler.py',
                    name='tag_leveler',
                    output='screen'
                )
            ]
        ),
    ])
