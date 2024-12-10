import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():


    pkg_share = FindPackageShare(package='jsc_mecanum_description').find('jsc_mecanum_description')
    default_model_path = os.path.join(pkg_share, 'src/jsc_mecanum_description.urdf')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    model = LaunchConfiguration('model')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')



    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            name='use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher'),
        
        DeclareLaunchArgument(
            name='model', 
            default_value=default_model_path, 
            description='Absolute path to robot urdf file'),
        Node(
            package= 'robot_state_publisher',
            executable= 'robot_state_publisher',
            name= 'robot_state_publisher',
            output='sceen',
            parameters=[{'user_sim_time': use_sim_time,
                         'robot_description': default_model_path}]
        ),
        # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
        Node(
            condition=IfCondition(use_robot_state_pub),
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 
                         'robot_description': Command(['xacro ', model])}],
            arguments=[default_model_path]
        )


    ])
