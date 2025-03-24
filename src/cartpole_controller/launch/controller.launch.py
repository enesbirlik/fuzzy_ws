import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    
    # Simülasyon modu argümanı
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="true",  # Varsayılan olarak simülasyon modu açık
        description="Run in simulation mode"
    )

    # Controller tipi argümanı
    controller_type_arg = DeclareLaunchArgument(
        "controller_type",
        default_value="position",  # Varsayılan olarak pozisyon kontrolü seçili
        description="Controller type: 'position' or 'velocity'"
    )

    # Launch parametrelerini oku
    is_sim = LaunchConfiguration("is_sim")
    controller_type = LaunchConfiguration("controller_type")

    # URDF dosyasını oluştur
    robot_description = ParameterValue(
        Command([
            "xacro",
            os.path.join(get_package_share_directory("cartpole_description"), "urdf", "cartpole.urdf.xacro"),
            " is_sim:=True"
        ]),
        value_type=str
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        condition=UnlessCondition(is_sim)
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
             "use_sim_time": PythonExpression(["'", is_sim, "' == 'true'"])},  # Doğru tipte değer döndürmesi için düzeltildi
            os.path.join(
                get_package_share_directory("cartpole_controller"),
                "config",
                "cartpole_controllers.yaml"
            )
        ],
        condition=UnlessCondition(is_sim)  # Sadece gerçek donanımda çalıştır
    )

    # Load Joint State Broadcaster (Her iki modda da çalıştır)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Load Position Controller (Eğer `controller_type` "position" ise çalıştır)
    load_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cart_position_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'position'"]))  # Düzeltildi
    )

    # Load Velocity Controller (Eğer `controller_type` "velocity" ise çalıştır)
    load_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "cart_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'velocity'"]))  # Düzeltildi
    )

    return LaunchDescription([
        # Launch Arguments
        is_sim_arg,
        controller_type_arg,
        
        # Launch Nodes
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_position_controller,
        load_velocity_controller,
    ])
