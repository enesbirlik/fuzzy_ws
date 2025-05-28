import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
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

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(cartpole_fuzzy_share, 'config', 'cartpole_controller_effort.yaml')
        ],
        output='screen',
    )

    # Load Controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'effort_controllers'],
        output='screen'
    )

    #rviz configuration /home/enesb/.rviz2
    rviz_config_dir = '/home/enesb/.rviz2/cartpole.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        
        # Launch Nodes
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_effort_controller,
        rviz_node
    ])