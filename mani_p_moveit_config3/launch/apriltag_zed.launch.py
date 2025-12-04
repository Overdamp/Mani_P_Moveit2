#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='zed',
        description='Name of the ZED camera node'
    )
    
    camera_name = LaunchConfiguration('camera_name')

    config_file = PathJoinSubstitution([
        FindPackageShare('mani_p_moveit_config3'),
        'config',
        'apriltag_tags.yaml'
    ])

    return LaunchDescription([
        camera_name_arg,
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('image_rect',    [camera_name, '/zed_node/rgb/color/rect/image']),
                ('camera_info',   [camera_name, '/zed_node/rgb/color/rect/camera_info']),
                ('image_rect/compressed', [camera_name, '/zed_node/rgb/color/rect/image/compressed']),
            ],
            parameters=[config_file]
        )
    ])