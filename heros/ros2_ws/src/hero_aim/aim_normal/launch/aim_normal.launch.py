from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    core_config = get_package_share_directory("aim_core") + "/config/ballistics.yaml"
    strategy_config = get_package_share_directory("aim_normal") + "/config/aim_normal.yaml"
    return LaunchDescription([
        Node(
            package="aim_normal",
            executable="aim_normal_node",
            name="aim_normal_node",
            output="screen",
            # 先载入共用物理参数，再载入普通模式自身参数。
            parameters=[core_config, strategy_config],
        )
    ])
