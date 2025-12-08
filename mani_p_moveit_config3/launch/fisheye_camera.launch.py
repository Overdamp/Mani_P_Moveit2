from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mani_p_moveit_config3',
            executable='fisheye_camera.py',
            name='fisheye_camera_node',
            output='screen',
            parameters=[{
                'device_id': 1,
                'frame_id': 'fisheye_camera_link',
                'width': 640,
                'height': 480,
                'fps': 30
            }]
        )
    ])
