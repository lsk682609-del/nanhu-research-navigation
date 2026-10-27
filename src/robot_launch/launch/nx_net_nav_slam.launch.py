"""Nav2 road-network navigation + FAST-LIVO2 localization + RoboSense LiDAR driver."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_launch")
    default_nav_params = os.path.join(pkg_share, "config", "nx_net_nav.yaml")
    default_rviz = os.path.join(pkg_share, "config", "nav2.rviz")

    rslidar_share = get_package_share_directory("rslidar_sdk")
    rslidar_launch_file = os.path.join(rslidar_share, "launch", "start.py")

    fast_livo_share = get_package_share_directory("fast_livo")
    default_livo_params = os.path.join(fast_livo_share, "config", "car1_airy_ros2.yaml")
    default_cam_params = os.path.join(fast_livo_share, "config", "camera_pinhole_mid360.yaml")

    # ---- declare arguments ----
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Override use_sim_time for launched nodes.",
    )
    declare_nav_params_file = DeclareLaunchArgument(
        "nav_params_file", default_value=default_nav_params,
        description="Nav2 multi-node YAML.",
    )
    declare_livo_params_file = DeclareLaunchArgument(
        "livo_params_file", default_value=default_livo_params,
        description="FAST-LIVO2 YAML config.",
    )
    declare_cam_params_file = DeclareLaunchArgument(
        "cam_params_file", default_value=default_cam_params,
        description="Camera YAML config.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="RViz2 config path.",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Start RViz2.",
    )
    declare_use_rslidar = DeclareLaunchArgument(
        "use_rslidar", default_value="true",
        description="Launch RoboSense LiDAR driver.",
    )

    # ---- resolve configurations ----
    nav_params_file = LaunchConfiguration("nav_params_file")
    livo_params_file = LaunchConfiguration("livo_params_file")
    cam_params_file = LaunchConfiguration("cam_params_file")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")
    use_rslidar = LaunchConfiguration("use_rslidar")

    sim_time_param = {"use_sim_time": use_sim_time}

    # ---- RoboSense LiDAR driver ----
    rslidar_driver = Node(
        package="rslidar_sdk",
        executable="rslidar_sdk_node",
        name="rslidar_sdk_node",
        output="screen",
        condition=IfCondition(use_rslidar),
    )

    # ---- TF: car1/base_link -> rslidar ----
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_rslidar_tf",
        arguments=["0.20", "0", "0.30", "0.0", "0.349", "0.0",
                   "base_link", "lidar"],
        parameters=[sim_time_param],
        output="screen",
    )

    # ---- FAST-LIVO2 SLAM node ----
    livo_node = Node(
        package="fast_livo",
        executable="fastlivo_mapping",
        name="car1",
        output="screen",
        parameters=[livo_params_file, cam_params_file],
    )

    # ---- Navigation stack ----
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    net_planner_action_node = Node(
        package="nav2_net_planner",
        executable="net_planner_action_server",
        name="net_planner_action_server",
        output="screen",
        parameters=[nav_params_file, sim_time_param,
            {"topo_map_path": os.path.join(pkg_share, "maps", "topo_map.yaml")}],
    )

    net_navigator_node = Node(
        package="nav2_net_system",
        executable="net_navigator_node",
        name="net_navigator",
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    input_bridge_node = Node(
        package="nav2_input_bridge",
        executable="input_bridge_node",
        name="nav2_input_bridge",
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    log_record_node = Node(
        package="nav2_log_record",
        executable="log_server_node",
        name="nav2_log_record",
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    # ---- CAN chassis driver ----
    car_driver_node = Node(
        package="ros2can",
        executable="car_driver",
        name="car_driver",
        output="screen",
    )

    # ---- RViz ----
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[sim_time_param],
        output="screen",
    )

    # ---- Lifecycle manager for navigation nodes ----
    lc_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_nav_params_file,
            declare_livo_params_file,
            declare_cam_params_file,
            declare_rviz_config,
            declare_use_rviz,
            declare_use_rslidar,
            rslidar_driver,
            static_tf_base_lidar,
            livo_node,
            controller_server_node,
            net_planner_action_node,
            net_navigator_node,
            input_bridge_node,
            car_driver_node,
            log_record_node,
            lc_manager_node,
            rviz_node,
        ]
    )
