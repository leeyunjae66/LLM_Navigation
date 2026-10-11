from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_task_planner',
            executable='planner',
            name='llm_planner',
            output='screen',
        ),
        Node(
            package='llm_task_planner',
            executable='executor',
            name='skill_executor',
            output='screen',
        ),
    ])
