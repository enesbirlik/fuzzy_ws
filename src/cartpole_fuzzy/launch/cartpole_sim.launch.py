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

    cartpole_fuzzy_path = os.path.join(
        get_package_share_directory('cartpole_fuzzy'))

    xacro_file = os.path.join(cartpole_fuzzy_path,
                              'urdf',
                              'cartpole.xacro.urdf')

    # Parse Xacro file
    with open(xacro_file, 'r') as file:
        doc = xacro.parse(file)
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', doc.toxml(),
                   '-name', 'cartpole',
                   '-allow_renaming', 'true',
                   '-J', 'cart_to_pole', '1.5'],
    )

    # Joint State Broadcaster
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Effort Controller
    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'effort_controllers'],
        output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', ' -r -v 4 /home/enesb/fuzzy_ws/src/cartpole_fuzzy/worlds/my_world.sdf')],
        ),
        
        # Register Event Handlers
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_effort_controller],
            )
        ),
        
        # Nodes
        node_robot_state_publisher,
        node_joint_state_publisher,
        ignition_spawn_entity,
        
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use simulated clock'),
    ])