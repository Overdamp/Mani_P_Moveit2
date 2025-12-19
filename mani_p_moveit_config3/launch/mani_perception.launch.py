from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions import FindPackageShare
import subprocess
import re

def find_camera_device_id(target_name_pattern="USB Camera"):
    """
    Finds the video device ID (e.g., '0', '2') for a camera matching the target_name_pattern.
    Defaults to '0' if not found.
    """
    try:
        # Run v4l2-ctl --list-devices
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        output = result.stdout
        
        # Parse output
        # Example output:
        # USB Camera: USB Camera (usb-0000:00:14.0-1):
        #         /dev/video0
        #         /dev/video1
        
        lines = output.split('\n')
        current_camera_name = ""
        
        for line in lines:
            if not line.startswith('\t') and line.strip():
                current_camera_name = line.strip()
            elif line.startswith('\t') and target_name_pattern in current_camera_name:
                device_path = line.strip()
                # Extract number from /dev/videoX
                match = re.search(r'/dev/video(\d+)', device_path)
                if match:
                    return match.group(1)
                    
    except Exception as e:
        print(f"Error finding camera: {e}")
        
    return '3' # Default fallback
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

        # 5. fisheye_camera (Wait 5s for Detections)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('fisheye_camera'),
                            'launch',
                            'camera_system.launch.py'
                        ])
                    ),
                    launch_arguments={
                        'device_id': find_camera_device_id("USB 2.0 Camera") # Change "USB 2.0 Camera" to your camera name
                    }.items()
                )
            ]
        ),
    ])
