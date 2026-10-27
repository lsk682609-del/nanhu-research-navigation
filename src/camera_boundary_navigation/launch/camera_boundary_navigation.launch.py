from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    enabled_on_startup = LaunchConfiguration("enabled_on_startup")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    default_params = PathJoinSubstitution(
        [FindPackageShare("camera_boundary_navigation"), "config", "boundary_follower.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "enabled_on_startup",
                default_value="false",
                description="Unsafe for unattended use: allow non-zero motion after inputs recover",
            ),
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/camera_navigation/cmd_vel",
                description="Keep isolated for validation; set /cmd_vel only for supervised vehicle tests",
            ),
            Node(
                package="camera_boundary_navigation",
                executable="camera_boundary_navigation_node",
                name="camera_boundary_navigation_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "motion.enabled_on_startup": enabled_on_startup,
                        "cmd_vel_topic": cmd_vel_topic,
                    },
                ],
            ),
        ]
    )
