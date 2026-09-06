from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("hero_antibase"), "config", "hero_antibase.yaml")
    return LaunchDescription([
        Node(
            package="hero_antibase",
            executable="hero_antibase_node",
            name="hero_antibase_node",
            output="screen",
            parameters=[config],
        )
    ])
