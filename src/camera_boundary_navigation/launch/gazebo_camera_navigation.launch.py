from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    enabled_on_startup = LaunchConfiguration("enabled_on_startup")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    perception_params_file = LaunchConfiguration("perception_params_file")
    navigation_params_file = LaunchConfiguration("navigation_params_file")
    gazebo_params_file = LaunchConfiguration("gazebo_params_file")

    default_perception_params = PathJoinSubstitution(
        [
            FindPackageShare("traversability_obstacle_cloud"),
            "config",
            "camera_ground_extraction.yaml",
        ]
    )
    default_navigation_params = PathJoinSubstitution(
        [FindPackageShare("camera_boundary_navigation"), "config", "boundary_follower.yaml"]
    )
    default_gazebo_params = PathJoinSubstitution(
        [
            FindPackageShare("camera_boundary_navigation"),
            "config",
            "gazebo_camera_navigation.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("enabled_on_startup", default_value="false"),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/camera_navigation/cmd_vel",
                description="Keep isolated until perception and stop behavior are verified",
            ),
            DeclareLaunchArgument(
                "perception_params_file", default_value=default_perception_params
            ),
            DeclareLaunchArgument(
                "navigation_params_file", default_value=default_navigation_params
            ),
            DeclareLaunchArgument("gazebo_params_file", default_value=default_gazebo_params),
            Node(
                package="traversability_obstacle_cloud",
                executable="depth_cloud_projector_node",
                name="depth_cloud_projector_node",
                output="screen",
                parameters=[
                    perception_params_file,
                    gazebo_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="traversability_obstacle_cloud",
                executable="traversability_obstacle_cloud_node",
                name="traversability_obstacle_cloud_node",
                output="screen",
                parameters=[
                    perception_params_file,
                    gazebo_params_file,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="camera_boundary_navigation",
                executable="camera_boundary_navigation_node",
                name="camera_boundary_navigation_node",
                output="screen",
                parameters=[
                    navigation_params_file,
                    gazebo_params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "motion.enabled_on_startup": enabled_on_startup,
                        "cmd_vel_topic": cmd_vel_topic,
                    },
                ],
            ),
            Node(
                package="traversability_obstacle_cloud",
                executable="camera_costmap_cloud_node",
                name="camera_costmap_cloud_node",
                output="screen",
                parameters=[
                    perception_params_file,
                    gazebo_params_file,
                    PathJoinSubstitution([FindPackageShare("camera_boundary_navigation"),
                                          "config", "processed_perception.yaml"]),
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
