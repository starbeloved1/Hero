from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory("gimbal_driver") + "/config/gimbal_driver.yaml"
    return LaunchDescription([
        Node(
            package="gimbal_driver",
            executable="gimbal_driver_node",
            name="gimbal_driver_node",
            output="screen",
            parameters=[config],
        )
    ])
