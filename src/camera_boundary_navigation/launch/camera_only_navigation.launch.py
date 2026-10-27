from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    depth_input_mode = LaunchConfiguration("depth_input_mode")
    use_rgb_decoder = LaunchConfiguration("use_rgb_decoder")
    enabled_on_startup = LaunchConfiguration("enabled_on_startup")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    perception_params_file = LaunchConfiguration("perception_params_file")
    navigation_params_file = LaunchConfiguration("navigation_params_file")

    perception_launch = PathJoinSubstitution(
        [
            FindPackageShare("traversability_obstacle_cloud"),
            "launch",
            "camera_single_frame_ground.launch.py",
        ]
    )
    follower_launch = PathJoinSubstitution(
        [
            FindPackageShare("camera_boundary_navigation"),
            "launch",
            "camera_boundary_navigation.launch.py",
        ]
    )
    default_perception_params = PathJoinSubstitution(
        [
            FindPackageShare("traversability_obstacle_cloud"),
            "config",
            "camera_ground_extraction.yaml",
        ]
    )
    default_navigation_params = PathJoinSubstitution(
        [
            FindPackageShare("camera_boundary_navigation"),
            "config",
            "boundary_follower.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "depth_input_mode",
                default_value="config",
                choices=["config", "raw", "compressed_depth"],
            ),
            DeclareLaunchArgument("use_rgb_decoder", default_value="false"),
            DeclareLaunchArgument("enabled_on_startup", default_value="false"),
            DeclareLaunchArgument(
                "perception_params_file", default_value=default_perception_params
            ),
            DeclareLaunchArgument(
                "navigation_params_file", default_value=default_navigation_params
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic", default_value="/camera_navigation/cmd_vel"
            ),
            # 两个子launch都声明了名为params_file的参数。必须使用独立作用域，
            # 否则感知参数文件会泄漏给跟随节点，使其回退到代码默认参数。
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(perception_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "depth_input_mode": depth_input_mode,
                            "use_rgb_decoder": use_rgb_decoder,
                            "params_file": perception_params_file,
                        }.items(),
                    )
                ],
            ),
            Node(
                package="traversability_obstacle_cloud",
                executable="camera_costmap_cloud_node",
                name="camera_costmap_cloud_node",
                output="screen",
                parameters=[
                    perception_params_file,
                    PathJoinSubstitution([FindPackageShare("camera_boundary_navigation"),
                                          "config", "processed_perception.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(follower_launch),
                        launch_arguments={
                            "use_sim_time": use_sim_time,
                            "enabled_on_startup": enabled_on_startup,
                            "cmd_vel_topic": cmd_vel_topic,
                            "params_file": navigation_params_file,
                        }.items(),
                    )
                ],
            ),
        ]
    )
