from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    core_share_directory = get_package_share_directory("aim_core")
    ballistics_config = core_share_directory + "/config/ballistics.yaml"
    strategy_config = get_package_share_directory("aim_antitop") + "/config/aim_antitop.yaml"
    return LaunchDescription([
        Node(
            package="aim_antitop",
            executable="aim_antitop_node",
            name="aim_antitop_node",
            output="screen",
            # 先载入公共弹道参数，再载入反前哨自身参数。
            parameters=[ballistics_config, strategy_config],
        )
    ])
