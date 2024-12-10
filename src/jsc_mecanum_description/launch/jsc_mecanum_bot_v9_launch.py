
#! Author: Juan Socas
#! Description: Launch a JSC mobile robot with 4 wheels

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():

    
    bringup_dir = get_package_share_directory('a_gazebo')
    launch_dir = os.path.join(bringup_dir, 'launch')
    pkg_share = FindPackageShare(package='jsc_mecanum_description').find('jsc_mecanum_description')
    pkg_hardware_bringup = FindPackageShare(package='jsc_mecanum_bringup').find('jsc_mecanum_bringup')
    default_model_path = os.path.join(pkg_share, 'src/jsc_mecanum_description.urdf')
    
    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')


    # Launch configuration variables specific to simulation
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Declare the launch arguments

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')
    

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/jsc/ros2_ws/src/a_gazebo/maps/mapa_2.yaml',
        description='Full path to map file to load')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')


    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='False',
        description='Whether to start the simulator')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file')



    #! ROS2_CONTROL --> JSC_MECANUM_DRIVE_CONTROLLER
    JSC_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_hardware_bringup , 'launch', 'jsc_mecanum_hardware.py')
            ),
        )

    # Specify the actions

    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace = namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        remappings=remappings,
        arguments=[default_model_path])


    #! LIDAR ONDOMETRY 
    rf2o_ondo = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry_1',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom',
            'publish_tf' : True,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 20.0}],
    )

    #! RVIZ
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'rviz_config': rviz_config_file}.items())

    #! NAV2
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())

    #! TF 
    TF_base_link__to__lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_to_base_link',
        output='screen',
        arguments=['0.05', '0', '0.085', '0', '0', '0', 'base_link', 'lidar_link'])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_map_yaml_cmd)


    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)



    
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)

    #! TF Necessary
    ld.add_action(TF_base_link__to__lidar_link)



    ld.add_action(rf2o_ondo)
    #ld.add_action(JSC_control)

    return ld