from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = get_package_share_directory("aim_auto") + "/config/aim_auto.yaml"
    core_share_directory = get_package_share_directory("aim_core")
    ballistics_file = core_share_directory + "/config/ballistics.yaml"
    smoother_file = core_share_directory + "/config/angle_smoother.yaml"
    return LaunchDescription([
        Node(
            package="aim_auto",
            executable="aim_auto_node",
            name="aim_auto_node",
            output="screen",
            parameters=[ballistics_file, smoother_file, config_file],
        )
    ])
