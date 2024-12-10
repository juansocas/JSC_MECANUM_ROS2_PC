from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                '/home/jsc/ros2_ws/src/robot_slam/config/slam_toolbox_config.yaml'
            ],
            remappings=[('/scan', '/scan')]
        )
    ])

