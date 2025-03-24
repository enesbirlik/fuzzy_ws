import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    controller = IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("cartpole_controller"),
                "launch",
                "controller.launch.py"
            ),
            launch_arguments={"is_sim": "False"}.items()
        )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory("cartpole_description"), "rviz", "display.rviz")]
    )
    
    # remote_interface = IncludeLaunchDescription(
    #         os.path.join(
    #             get_package_share_directory("cartpole_remote"),
    #             "launch",
    #             "remote_interface.launch.py"
    #         ),
    #     )
    
    return LaunchDescription([
        controller,
        rviz_node,
        #remote_interface,
    ])