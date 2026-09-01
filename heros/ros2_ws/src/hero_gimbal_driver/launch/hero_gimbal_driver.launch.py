from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory("hero_gimbal_driver") + "/config/hero_gimbal_driver.yaml"
    return LaunchDescription([
        Node(
            package="hero_gimbal_driver",
            executable="hero_gimbal_driver_node",
            name="hero_gimbal_driver_node",
            output="screen",
            parameters=[config],
        )
    ])
