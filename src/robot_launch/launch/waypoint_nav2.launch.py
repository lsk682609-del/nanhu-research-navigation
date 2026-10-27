# Copyright 2026
# Author: Wei Wang -> shaxikai@outlook.com
#
# Waypoint-based navigation: direct path following without global planner.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory("robot_launch")
    default_params = os.path.join(pkg_share, "config", "waypoint_nav2.yaml")
    default_rviz = os.path.join(pkg_share, "config", "nav2.rviz")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Override use_sim_time for launched nodes.",
    )
    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="Nav2-style multi-node YAML.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="RViz2 config path.",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz2.",
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    sim_time_param = {"use_sim_time": use_sim_time}

    static_tf_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_odom_tf",
        arguments=["0", "0", "0", "0", "0", "0", "1", "map", "odom"],
        parameters=[params_file, sim_time_param],
        output="screen",
    )

    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file],
    )

    waypoint_navigator_node = Node(
        package="nav2_waypoint_system",
        executable="waypoint_navigator_node",
        name="waypoint_navigator",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    log_record_node = Node(
        package="nav2_log_record",
        executable="log_server_node",
        name="nav2_log_record",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[sim_time_param],
        output="screen",
    )

    lc_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[params_file, sim_time_param],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_params_file,
            declare_rviz_config,
            declare_use_rviz,
            static_tf_map_odom,
            controller_server_node,
            waypoint_navigator_node,
            log_record_node,
            lc_manager_node,
            rviz_node,
        ]
    )