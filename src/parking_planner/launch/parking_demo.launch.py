from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("parking_planner"))
    rviz_config = str(package_share / "rviz" / "parking_demo.rviz")

    return LaunchDescription(
        [
            Node(
                package="parking_planner",
                executable="parking_node",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
