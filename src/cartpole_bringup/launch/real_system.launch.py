import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # cartpole_controller paketindeki controller.launch.py dosyasının tam yolunu alıyoruz
    controller_launch_file = os.path.join(
        get_package_share_directory("cartpole_controller"),
        "launch",
        "controller.launch.py"
    )
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controller_launch_file),
        launch_arguments={"is_sim": "False"}.items()
    )
    
    # RViz için konfigürasyon dosyasının yolunu belirliyoruz
    rviz_config_file = os.path.join(
        get_package_share_directory("cartpole_description"),
        "rviz",
        "display.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )
    
    return LaunchDescription([
        controller,
        rviz_node,
    ])
