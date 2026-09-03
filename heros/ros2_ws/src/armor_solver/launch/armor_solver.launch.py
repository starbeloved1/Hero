from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=get_package_share_directory("armor_solver") + "/config/armor_solver.yaml",
            description="位姿解算节点参数文件路径",
        ),
        Node(
            package="armor_solver",
            executable="armor_solver_node",
            name="armor_solver_node",
            output="screen",
            parameters=[config],
        )
    ])
