from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_path = get_package_share_directory("hero_tf") + "/config/hero_tf.yaml"
    return LaunchDescription([
        Node(
            package="hero_tf",
            executable="hero_tf_node",
            name="hero_tf_node",
            output="screen",
            parameters=[config_path],
        )
    ])
