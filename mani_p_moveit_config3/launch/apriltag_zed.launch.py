#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('mani_p_moveit_config3'),
        'config',
        'apriltag_tags.yaml'
    ])

    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('image_rect',    '/zed/zed_node/rgb/color/rect/image'),
                ('camera_info',   '/zed/zed_node/rgb/color/rect/camera_info'),
                ('image_rect/compressed', '/zed/zed_node/rgb/color/rect/image/compressed'),
            ],
            parameters=[config_file]
        )
    ])