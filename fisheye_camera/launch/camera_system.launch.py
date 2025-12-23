from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.substitutions import FindPackageShare
import subprocess
import re
import os

def find_camera_device_id(target_name_pattern="USB Camera"):
    """
    Finds the video device ID (e.g., '0', '2') or path (e.g., '/dev/fisheye_camera').
    Prioritizes UDEV symlink '/dev/fisheye_camera'.
    """
    # 1. Check for UDEV symlink
    if os.path.exists('/dev/fisheye_camera'):
        return '/dev/fisheye_camera'

    # 2. Fallback to v4l2-ctl search
    try:
        # Run v4l2-ctl --list-devices
        result = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True)
        output = result.stdout
        
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
        
    return '2' # Default fallback
def generate_launch_description():
    pkg_share = FindPackageShare('fisheye_camera')
    
    # Arguments
    # Arguments
    # Auto-detect device ID, default to 3 if not found
    default_device_id = find_camera_device_id("USB 2.0 Camera")
    device_id_arg = DeclareLaunchArgument('device_id', default_value=default_device_id)
    width_arg = DeclareLaunchArgument('width', default_value='640')
    height_arg = DeclareLaunchArgument('height', default_value='480')
    fps_arg = DeclareLaunchArgument('fps', default_value='15')
    rotation_arg = DeclareLaunchArgument('rotation', default_value='180')
    
    calibration_file = PathJoinSubstitution([
        pkg_share, 'config', 'calibration.yaml'
    ])

    return LaunchDescription([
        device_id_arg,
        width_arg,
        height_arg,
        fps_arg,
        rotation_arg,

        # 1. Driver Node
        Node(
            package='fisheye_camera',
            executable='fisheye_camera.py', # Script name
            name='fisheye_camera_node',
            output='screen',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'rotation': LaunchConfiguration('rotation'),
                'calibration_file': calibration_file
            }],
            remappings=[
                ('image_raw', '/fisheye_camera/image_raw'),
                ('camera_info', '/fisheye_camera/camera_info')
            ]
        ),

        # 2. Image Proc (Rectification)
        Node(
            package='image_proc',
            executable='image_proc',
            name='image_proc',
            output='screen',
            remappings=[
                ('image', '/fisheye_camera/image_raw'),
                ('camera_info', '/fisheye_camera/camera_info'),
                ('image_rect', '/image_rect'),
                ('image_rect_color', '/image_rect_color')
            ],
            parameters=[
                {'image_transport': 'raw'}
            ]
        ),

        # 3. AprilTag Node
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node_fisheye',
            output='screen',
            remappings=[
                ('image_rect', '/image_rect'),
                ('camera_info', '/fisheye_camera/camera_info')
            ],
            parameters=[
                PathJoinSubstitution([FindPackageShare('mani_p_moveit_config3'), 'config', 'apriltag_tags_fisheye.yaml']),
                {'image_transport': 'raw'} # Override to use raw image from image_proc
            ]
        ),

        # 4. Tag Leveler (Force Z-axis parallel to floor)
        # Added to fix tilted AprilTag frames
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='fisheye_camera',
                    executable='fisheye_tag_leveler.py',
                    name='fisheye_tag_leveler',
                    output='screen'
                )
            ]
        )
    ])
