from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("plan_env")) / "config" / "depth_grid_map.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("extrinsic_config", default_value=default_config),
        Node(
            package="plan_env",
            executable="grid_map_node",
            name="grid_map_node",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                LaunchConfiguration("extrinsic_config"),
            ],
        ),
    ])
