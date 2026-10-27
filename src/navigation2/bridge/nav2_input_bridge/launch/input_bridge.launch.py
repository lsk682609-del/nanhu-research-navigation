"""Start nav2_input_bridge with package defaults (override params_file if needed)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("nav2_input_bridge")
    pkg_share_robot = get_package_share_directory("robot_launch")
    default_params = os.path.join(pkg, "config", "defaults.yaml")
    common_params = os.path.join(pkg_share_robot, "config", "common_params.yaml")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Set use_sim_time on the bridge node.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="YAML with nav2_input_bridge ros__parameters.",
    )

    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    params_file = LaunchConfiguration("params_file")

    bridge = Node(
        package="nav2_input_bridge",
        executable="input_bridge_node",
        name="nav2_input_bridge",
        output="screen",
        parameters=[params_file, common_params, {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            bridge,
        ]
    )
