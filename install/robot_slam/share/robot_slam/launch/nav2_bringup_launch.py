from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            output='screen',
            parameters=['config/nav2_params.yaml'],
        ),
    ])
