# Author: Juan Socas
# Description: Launch a JSC mobile robot with 4 wheels


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
  pkg_share = FindPackageShare(package='jsc_mobile_robot').find('jsc_mobile_robot')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'models/jsc_mobile_bot_v2_copy.urdf')
  
  ####
  diff_drive_controller_file_path = os.path.join(pkg_share,'config/diff_drive_controller.yaml')

  robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 
  robot_name_in_urdf = 'jsc_mobile_bot'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
  world_file_name ='jsc_mobile_bot_world/pasillo_telematica.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  
  # Launch configuration variables specific to simulation
  headless = LaunchConfiguration('headless')
  model = LaunchConfiguration('model')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')

  # Declare the launch arguments  
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')
    
  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
   
  # Specify the actions

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())
  
  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
  
  # Start robot localization using an Extended Kalman filter
  start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])
  

  # Load diff steer controller
  load_diff_drive_controller = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['diff_drive_controller'],
    output='screen',
    parameters=[diff_drive_controller_file_path, 
    {'use_sim_time': use_sim_time}])

# Nodos adicionales para controladores
  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    output='screen')

  # Start teleop for keyboard control
  teleop_twist_keyboard_node = Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    name='teleop_twist_keyboard',
    output='screen',
    prefix='xterm -e',  # Esto abrirá un terminal separado para control manual
    remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
    parameters=[{'scale_linear': 0.5, 'scale_angular': 1.0}]
)
  
  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    arguments=[default_model_path])
  
  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)

  #
  # 
  ld.add_action(joint_state_broadcaster_spawner)
  ld.add_action(load_diff_drive_controller)
  ld.add_action(teleop_twist_keyboard_node)

  ##
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  # Add any actions
  ld.add_action(start_gazebo_server_cmd)
  ld.add_action(start_gazebo_client_cmd)
  ld.add_action(start_robot_localization_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)

  return ld