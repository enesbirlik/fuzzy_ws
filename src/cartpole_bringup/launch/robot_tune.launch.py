import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Resolve the full path to the world file
    world_path = os.path.join(
        get_package_share_directory('cartpole_bringup'),
        'worlds',
        'my_world.sdf'
    )

    # Declare launch argument for the world file
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_path,
        description='Full path to the world file to load in Gazebo'
    )




    env_variable = SetEnvironmentVariable(
    "GAZEBO_MODEL_PATH",
    os.path.join(get_package_share_directory("cartpole_description"), "models")
    )

    model_arg = DeclareLaunchArgument(
    "model",
    default_value=os.path.join(get_package_share_directory("cartpole_description"), "robot", "cartpole.urdf.xacro"),
    description="Absolute path to the robot URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]))

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )


    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py"
    )))

    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py"
    )))

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "cartpole", "-topic", "robot_description"],
        output="screen"
    )

    commander_node = Node(
        package="cartpole_bringup",
        executable="optimize_pid.py",
        name="commander_node",
        output="screen"
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cartpole_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True", "controller_type": "effort"}.items()
    )

    return LaunchDescription([
        world_arg,

        env_variable,
        model_arg,
        robot_state_publisher,

        start_gazebo_server,
        start_gazebo_client,

        spawn_robot,
        # commander_node,
        controller,
    ])  
