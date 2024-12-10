#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Directory paths
    bringup_dir = os.path.join(os.getenv('HOME'), 'ros2_ws', 'src', 'nav2_bringup', 'bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'))

    # Declare launch arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_argument = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Nodes to launch
    start_nav2_cmd = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        output='screen',
        parameters=[params_file])

    ld = LaunchDescription()

    # Add launch options
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_argument)

    # Add actions to launch Nav2
    ld.add_action(start_nav2_cmd)

    return ld