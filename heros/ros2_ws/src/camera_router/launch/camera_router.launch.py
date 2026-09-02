from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = get_package_share_directory("camera_router") + "/config/camera_router.yaml"
    return LaunchDescription([
        Node(
            package="camera_router",
            executable="camera_router_node",
            name="camera_router_node",
            output="screen",
            parameters=[config_path],
        )
    ])
