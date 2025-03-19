import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get package share directory as a string
    cartpole_fuzzy_share = get_package_share_directory('cartpole_fuzzy')
    
    # URDF/XACRO file path (full string path)
    xacro_path = os.path.join(cartpole_fuzzy_share, 'urdf', 'cartpole.xacro.urdf')
    
    # Parse XACRO file
    robot_description_content = xacro.process_file(xacro_path).toxml()
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Spawn entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'cartpole'
        ],
    )

    # Load joint state broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Load effort controllers
    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'effort_controllers'],
        output='screen'
    )

    # Launch Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 
                'launch', 'gazebo.launch.py')
        ]),
        launch_arguments=[
            ('verbose', 'false')
        ]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        # Launch Gazebo
        gazebo,
        
        # Launch Nodes
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
        
        # Event Handlers for Controller Loading
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_effort_controller],
            )
        ),
    ])