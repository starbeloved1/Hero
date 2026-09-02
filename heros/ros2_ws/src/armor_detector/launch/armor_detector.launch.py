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
            default_value=get_package_share_directory("armor_detector") + "/config/armor_detector.yaml",
            description="检测节点参数文件路径",
        ),
        Node(
            package="armor_detector",
            executable="armor_detector_node",
            name="armor_detector_node",
            output="screen",
            parameters=[config],
        )
    ])
