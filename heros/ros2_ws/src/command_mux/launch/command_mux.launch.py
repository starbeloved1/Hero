from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory("command_mux") + "/config/command_mux.yaml"
    return LaunchDescription([
        Node(
            package="command_mux",
            executable="command_mux_node",
            name="command_mux_node",
            output="screen",
            parameters=[config],
        )
    ])
