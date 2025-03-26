import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    controller_type_arg = DeclareLaunchArgument(
        'controller_type',
        default_value='velocity',
        description='Controller type: position or velocity or effort'
    )
    controller_type = LaunchConfiguration("controller_type")

    gazebo = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("cartpole_description"),
                "launch",
                "gazebo.launch.py"
            )
        )
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("cartpole_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={"is_sim": "True", "controller_type": controller_type}.items()
    )
    
    return LaunchDescription([
        controller_type_arg,
        gazebo,
        controller,
    ])