"""Nav2 road-network navigation without SLAM/LiDAR (minimal bringup)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("robot_launch")
    default_nav_params = os.path.join(pkg_share, "config", "nx_net_nav.yaml")
    default_rviz = os.path.join(pkg_share, "config", "nav2.rviz")

    # ---- declare arguments ----
    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="car1",
        description="Robot namespace.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Override use_sim_time for launched nodes.",
    )
    declare_nav_params_file = DeclareLaunchArgument(
        "nav_params_file", default_value=default_nav_params,
        description="Nav2 multi-node YAML.",
    )
    declare_topo_map_path = DeclareLaunchArgument(
        "topo_map_path", default_value="/root/jkl_car8/map/topo_map.yaml",
        description="Absolute path to the road-network topology YAML.",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config", default_value=default_rviz,
        description="RViz2 config path.",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz", default_value="true",
        description="Start RViz2.",
    )

    # ---- resolve configurations ----
    ns = LaunchConfiguration("namespace")
    nav_params_file = LaunchConfiguration("nav_params_file")
    topo_map_path = LaunchConfiguration("topo_map_path")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_config = LaunchConfiguration("rviz_config")
    use_rviz = LaunchConfiguration("use_rviz")

    sim_time_param = {"use_sim_time": use_sim_time}

    # ---- namespace-aware topic paths ----
    cur_lid_absolute = PythonExpression(["'/", ns, "/cur_lid'"])
    tf_topic = PythonExpression(["'/", ns, "/tf'"])
    tf_static_topic = PythonExpression(["'/", ns, "/tf_static'"])

    # ---- frame IDs (shared across cars, isolated by TF topic namespace) ----
    global_frame = "map"
    base_frame  = "base_link"
    lidar_frame = "lidar"

    # ---- static TF: base_link -> lidar ----
    static_tf_base_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_lidar_tf",
        namespace=ns,
        arguments=["0.24", "0", "0.30", "0", "0.0", "0",
                   base_frame, lidar_frame],
        parameters=[sim_time_param],
        remappings=[("/tf_static", "tf_static")],
        output="screen",
    )

    # ---- Navigation stack ----
    controller_server_node = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        namespace=ns,
        output="screen",
        parameters=[nav_params_file, sim_time_param],
        remappings=[
            ("cur_lid", cur_lid_absolute),
            ("/tf", tf_topic),
            ("/tf_static", tf_static_topic),
        ],
    )

    net_planner_action_node = Node(
        package="nav2_net_planner",
        executable="net_planner_action_server",
        name="net_planner_action_server",
        namespace=ns,
        output="screen",
        parameters=[
            nav_params_file,
            sim_time_param,
            {"topo_map_path": topo_map_path},
        ],
        remappings=[
            ("/tf", tf_topic),
            ("/tf_static", tf_static_topic),
        ],
    )

    net_navigator_node = Node(
        package="nav2_net_system",
        executable="net_navigator_node",
        name="net_navigator",
        namespace=ns,
        output="screen",
        parameters=[nav_params_file, sim_time_param],
        remappings=[
            ("/tf", tf_topic),
            ("/tf_static", tf_static_topic),
        ],
    )

    input_bridge_node = Node(
        package="nav2_input_bridge",
        executable="input_bridge_node",
        name="nav2_input_bridge",
        namespace=ns,
        output="screen",
        parameters=[nav_params_file, sim_time_param],
        remappings=[("/tf", "tf")],
    )

    log_record_node = Node(
        package="nav2_log_record",
        executable="log_server_node",
        name="nav2_log_record",
        namespace=ns,
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    # ---- CAN chassis driver (co-search) ----
    car_driver_node = Node(
        package="ros2can",
        executable="car_driver",
        name="car_driver",
        namespace=ns,
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

    # ---- Lifecycle manager ----
    lc_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        namespace=ns,
        output="screen",
        parameters=[nav_params_file, sim_time_param],
    )

    return LaunchDescription(
        [
            declare_namespace,
            declare_use_sim_time,
            declare_nav_params_file,
            declare_topo_map_path,
            declare_rviz_config,
            declare_use_rviz,
            static_tf_base_lidar,
            controller_server_node,
            net_planner_action_node,
            net_navigator_node,
            input_bridge_node,
            log_record_node,
            car_driver_node,
            lc_manager_node,
            rviz_node,
        ]
    )
