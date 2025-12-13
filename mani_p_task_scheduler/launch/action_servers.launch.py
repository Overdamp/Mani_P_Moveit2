from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mani_p_task_scheduler',
            executable='approach_action_server.py',
            name='approach_action_server',
            output='screen'
        ),
        Node(
            package='mani_p_task_scheduler',
            executable='adjust_level_action_server.py',
            name='adjust_level_action_server',
            output='screen'
        ),
        Node(
            package='mani_p_task_scheduler',
            executable='move_linear_action_server.py',
            name='move_linear_action_server',
            output='screen'
        ),
        Node(
            package='mani_p_task_scheduler',
            executable='gripper_action_server.py',
            name='gripper_action_server',
            output='screen'
        ),
        Node(
            package='mani_p_task_scheduler',
            executable='shelf_action_server.py',
            name='shelf_action_server',
            output='screen'
        ),
        Node(
            package='mani_p_task_scheduler',
            executable='move_to_named_target_action_server.py',
            name='move_to_named_target_action_server',
            output='screen'
        ),
    ])
