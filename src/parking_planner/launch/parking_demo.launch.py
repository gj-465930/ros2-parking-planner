from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("parking_planner"))
    rviz_config = str(package_share / "rviz" / "parking_demo.rviz")

    scenario = LaunchConfiguration("scenario")
    scenario_file = PathJoinSubstitution([str(package_share), "scenarios", scenario])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "scenario",
                default_value="perpendicular_parking.yaml",
                choices=[
                    "perpendicular_parking.yaml",
                    "parallel_parking.yaml",
                    "narrow_passage.yaml",
                ],
                description="选择要显示的泊车场景",
            ),
            Node(
                package="parking_planner",
                executable="parking_node",
                parameters=[{"scenario_file": scenario_file}],
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
