from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('fisheye_camera')
    
    # Arguments
    device_id_arg = DeclareLaunchArgument('device_id', default_value='3')
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
            name='apriltag_node',
            output='screen',
            remappings=[
                ('image_rect', '/fisheye_camera/image_raw'),
                ('camera_info', '/fisheye_camera/camera_info')
            ],
            parameters=[
                PathJoinSubstitution([FindPackageShare('mani_p_moveit_config3'), 'config', 'apriltag_tags_fisheye.yaml']),
                {'image_transport': 'raw'} # Override to use raw image from image_proc
            ]
        )
    ])
