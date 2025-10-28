import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
import xacro

def generate_launch_description():

    # Argomento per selezionare il controller
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="position",
        description="Type of controller to load: 'position' or 'trajectory'"
    )
    controller_type = LaunchConfiguration("controller_type")

    # Percorsi pacchetti
    pkg_armando_description = get_package_share_directory('armando_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # URDF
    urdf_file_name = "armando.urdf.xacro"
    urdf_file_path = os.path.join(pkg_armando_description, 'urdf', urdf_file_name)
    robot_description_config = xacro.process_file(urdf_file_path).toxml()

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config,
                     'use_sim_time': True}] 
    )

    # Simulatore Gazebo / Ignition
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Spawn robot
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_config,
                   '-name', 'armando',                
                   '-allow_renaming', 'true']
    )

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", 
                   "--controller-manager", "/controller_manager"],
    )

    # Position controller (solo se selezionato)
    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", 
                   "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'position'"]))
    )

    # Trajectory controller (solo se selezionato)
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", 
                   "--controller-manager", "/controller_manager"],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'trajectory'"]))
    )
    
    # Event handlers per caricamento controller
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    load_position_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[position_controller_spawner],
        )
    )
    load_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_node,
            on_exit=[trajectory_controller_spawner],
        )
    )

    # Bridge per camera (opzionale)
    bridge_camera = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )

    return LaunchDescription([
        controller_type_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity_node,
        load_joint_state_broadcaster,
        load_position_controller,
        load_trajectory_controller,
        bridge_camera
    ])
