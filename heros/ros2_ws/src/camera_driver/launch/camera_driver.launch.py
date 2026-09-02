from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_path = LaunchConfiguration("config_file")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=get_package_share_directory("camera_driver") + "/config/camera_driver.yaml",
            description="相机驱动参数文件路径",
        ),
        Node(
            package="camera_driver",
            executable="camera_driver_node",
            name="camera_driver_node",
            output="screen",
            parameters=[config_path],
        )
    ])
