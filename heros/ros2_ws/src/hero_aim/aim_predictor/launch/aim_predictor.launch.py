from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = get_package_share_directory("aim_predictor") + "/config/aim_predictor.yaml"
    return LaunchDescription([
        Node(
            package="aim_predictor",
            executable="aim_predictor_node",
            name="aim_predictor_node",
            output="screen",
            parameters=[config_file],
        )
    ])
